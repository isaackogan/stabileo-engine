//! THE COUPLED SOLVE — one load event, start to converged.
//!
//! Ported literally from the application's `cell-executor.ts` support search
//! (`solveSupportedFrame`) and per-event coupling loop (`solveEvent`), minus
//! what stays application-side by design: planning, per-event material
//! adjustment, initial compilation, structural criteria recovery, and every
//! identity hash. The kernel is invoked natively per round through
//! [`KernelPort`]; nothing is serialized between rounds.

use std::collections::{HashMap, HashSet};

use crate::partition::{advance_unit_load_partition, service_event_value, solve_unit_load_partition};
use crate::pallet::compare_canonical_utf8;
use crate::pallet::projection::{project_package_contacts, PalletMemberMap};
use crate::pallet::top_response::recover_pallet_top_response;
use crate::schema::{
    AnalysisFrame, CompiledPalletSupportState, FrameLoad, KernelResult,
    NumericalAcceptanceProfile, PalletContactProjectionResult, PalletTopResponse,
    UnitLoadActiveState,
};
use crate::support_state::{
    pallet_slip_friction_loads, restick_converged_pallet_support_state,
    update_pallet_support_active_state, ActiveStateChange, SupportUpdate,
};
use crate::types::{PalletError, PalletResult, Vec3};

pub use crate::kernel_bridge::FrameResponseRecovery;

/// The solver seam the loop drives. Bound to the native kernel bridge in the
/// WASM entry; bound to fakes in tests.
pub trait KernelPort {
    fn solve(
        &mut self,
        frame: &AnalysisFrame,
        request_id: &str,
        active_state_id: &str,
    ) -> PalletResult<KernelResult>;

    /// Beam-station recovery for the LAST solve this port performed. Station
    /// counts follow the reference: three per element.
    fn recover(
        &mut self,
        frame: &AnalysisFrame,
        kernel_result: &KernelResult,
        stations_per_element: u32,
    ) -> PalletResult<FrameResponseRecovery>;
}

/// Progress out of the loop, in the solve's own coordinates. The consumer
/// (the worker adapter) supplies event index/count context; everything
/// emitted here is what only the loop can know.
#[derive(Debug, Clone, serde::Serialize)]
#[serde(tag = "kind", rename_all = "camelCase")]
pub enum ProgressEmission {
    #[serde(rename_all = "camelCase")]
    Phase {
        phase: &'static str,
        coupling_round: u32,
        total: u32,
        share_residual: Option<f64>,
        translation_residual_m: Option<f64>,
        rotation_residual_rad: Option<f64>,
    },
    /// The `[mechanics]` console lines the reference printed — the user's
    /// window into the walk.
    #[serde(rename_all = "camelCase")]
    Note { message: String },
}

pub type ProgressSink<'a> = &'a mut dyn FnMut(&ProgressEmission);

/// JS `Number.prototype.toPrecision`, approximated for diagnostic sentences.
/// (Messages are pattern-matched by substring downstream, not byte-compared;
/// the numbers themselves are exact, only their rendering is approximate.)
fn to_precision(value: f64, digits: usize) -> String {
    if value == 0.0 || !value.is_finite() {
        return format!("{value}");
    }
    let magnitude = value.abs().log10().floor() as i32;
    if magnitude < -7 || magnitude >= digits as i32 {
        format!("{:.*e}", digits.saturating_sub(1), value)
    } else {
        let decimals = (digits as i32 - 1 - magnitude).max(0) as usize;
        format!("{value:.decimals$}")
    }
}

/// The census line every refusal carries: which normal/tangential classes the
/// supports were in when the model was refused.
fn support_census(support_state: &CompiledPalletSupportState) -> String {
    let mut parts: Vec<String> = Vec::new();
    for normal in ["ACTIVE", "LIFTED_OFF"] {
        for tangential in ["STICK", "SLIP", "INACTIVE"] {
            let count = support_state
                .active_state
                .iter()
                .filter(|state| state.normal_state == normal && state.tangential_state == tangential)
                .count();
            if count > 0 {
                parts.push(format!("{normal}/{tangential}x{count}"));
            }
        }
    }
    parts.join(" ")
}

fn kernel_diagnostics_sentence(kernel_result: &KernelResult) -> String {
    let mut counts: Vec<(String, u32)> = Vec::new();
    for entry in &kernel_result.diagnostics {
        let key = format!(
            "{}:{}@{} {}",
            entry.severity.to_lowercase(),
            entry.code.to_lowercase(),
            entry.entity_id.as_deref().unwrap_or("model"),
            entry.message
        );
        match counts.iter_mut().find(|(existing, _)| *existing == key) {
            Some((_, count)) => *count += 1,
            None => counts.push((key, 1)),
        }
    }
    let rendered: Vec<String> = counts
        .into_iter()
        .map(|(key, count)| if count > 1 { format!("{key} x{count}") } else { key })
        .collect();
    if rendered.is_empty() { "nothing".into() } else { rendered.join(" ; ") }
}

/// TS `composeSolvedPalletFrame` — the base frame with this round's support
/// rows, the projection's loads, the rigid face system, and the slipping
/// contacts' μN friction loads, all in canonical order. A SLIPPING CONTACT IS
/// A LOAD, not a silence, and the friction loads are composed HERE, with the
/// supports they belong to, so a frame can never carry one set without the
/// other.
pub fn compose_solved_pallet_frame(
    base_frame: &AnalysisFrame,
    support_state: &CompiledPalletSupportState,
    projected_loads: &[FrameLoad],
    face_system: Option<&crate::schema::ContactFaceSystem>,
) -> AnalysisFrame {
    let empty = crate::schema::ContactFaceSystem {
        nodes: vec![],
        constraints: vec![],
        connectors: vec![],
        supports: vec![],
        loads: vec![],
    };
    let face = face_system.unwrap_or(&empty);
    let mut nodes = base_frame.nodes.clone();
    nodes.extend(face.nodes.iter().cloned());
    nodes.sort_by(|left, right| compare_canonical_utf8(&left.node_id, &right.node_id));
    let mut constraints = base_frame.constraints.clone();
    constraints.extend(face.constraints.iter().cloned());
    constraints.sort_by(|left, right| {
        let id_of = |value: &serde_json::Value| {
            value
                .get("constraintId")
                .and_then(|id| id.as_str())
                .unwrap_or("")
                .to_string()
        };
        compare_canonical_utf8(&id_of(left), &id_of(right))
    });
    let mut connectors = base_frame.connectors.clone();
    connectors.extend(face.connectors.iter().cloned());
    connectors.sort_by(|left, right| compare_canonical_utf8(&left.connector_id, &right.connector_id));
    let mut supports = support_state.frame_supports.clone();
    supports.extend(face.supports.iter().cloned());
    supports.sort_by(|left, right| compare_canonical_utf8(&left.support_id, &right.support_id));
    let mut loads = base_frame.loads.clone();
    loads.extend(projected_loads.iter().cloned());
    loads.extend(face.loads.iter().cloned());
    loads.extend(pallet_slip_friction_loads(support_state));
    loads.sort_by(|left, right| compare_canonical_utf8(left.load_id(), right.load_id()));
    AnalysisFrame {
        nodes,
        constraints,
        connectors,
        supports,
        loads,
        // Application identity; internal rounds bind by construction.
        frame_hash: "internal".into(),
        ..base_frame.clone()
    }
}

pub struct SupportedSolve {
    pub support_state: CompiledPalletSupportState,
    pub solved_frame: AnalysisFrame,
    pub kernel_result: KernelResult,
    pub pallet_response: PalletTopResponse,
    pub bistable_contacts: Vec<(String, f64)>,
}

/// TS `solveSupportedFrame` — the support active-state search inside one
/// coupling round, with THE BISTABLE FREEZE: a pure two-cycle is a contact
/// exactly AT the bearing boundary, which a discrete active set cannot
/// express. When the detected cycle's recorded active-pole tension is under
/// the equilibrium audit's OWN force resolution for this frame — max(1 mN,
/// 1e-3 × applied force) — the support is frozen in its ACTIVE pole, its
/// tension DISCLOSED, and the search continues without it. A cycle LOUDER
/// than the solve's own resolution is a real modeling failure and still
/// refuses.
#[allow(clippy::too_many_arguments)]
pub fn solve_supported_frame(
    kernel: &mut dyn KernelPort,
    base_frame: &AnalysisFrame,
    member_map: &PalletMemberMap,
    initial_support_state: &CompiledPalletSupportState,
    unit_state: &UnitLoadActiveState,
    pallet_overall_m: (f64, f64),
    profile: &NumericalAcceptanceProfile,
    event_id: &str,
    coupling_iteration: u32,
    coupling_context: &CouplingProgressContext,
    progress: ProgressSink<'_>,
) -> PalletResult<SupportedSolve> {
    let partition = solve_unit_load_partition(unit_state, &service_event_value(unit_state)?, profile)?;
    let projection: PalletContactProjectionResult = project_package_contacts(
        base_frame,
        member_map,
        &partition.pallet_contacts,
        profile,
    )?;
    let mut support_state = initial_support_state.clone();
    // What the last round of the search was still doing, for the message it
    // throws if it never settles.
    let mut last_changes: Vec<ActiveStateChange> = Vec::new();
    let mut last_residual_n: f64 = 0.0;
    // THE BISTABLE FREEZE bookkeeping. Detection is structural: the exact
    // inverse of the immediately preceding pivot, or a support whose CONTACT
    // state is proposed to change for the third time — twice could be a
    // legitimate transient healed by redistribution, three cannot.
    let mut bistable_frozen: HashMap<String, f64> = HashMap::new();
    let mut previous_pivot: Option<ActiveStateChange> = None;
    let mut contact_toggle_count: HashMap<String, u32> = HashMap::new();
    // Each ACTIVE support's latest measured tension: a cycling contact
    // re-records this every period, so the record IS the cycle's own live
    // amplitude even when the freeze fires at the LIFTED pole.
    let mut pole_tension_n: HashMap<String, f64> = HashMap::new();
    let applied_force_norm_n = {
        let mut total = Vec3::ZERO;
        for load in &base_frame.loads {
            if let FrameLoad::NodalForce { force, .. } = load {
                total.x += force.x;
                total.y += force.y;
                total.z += force.z;
            }
        }
        for load in projection.loads.iter().chain(projection.face_system.loads.iter()) {
            if let FrameLoad::NodalForce { force, .. } = load {
                total.x += force.x;
                total.y += force.y;
                total.z += force.z;
            }
        }
        total.hypot3()
    };
    let bistable_admissible_n = (0.001_f64).max(0.001 * applied_force_norm_n);
    for support_iteration in 0..profile.coupled_iteration_limit {
        let solved_frame = compose_solved_pallet_frame(
            base_frame,
            &support_state,
            &projection.loads,
            Some(&projection.face_system),
        );
        // The COUPLING round is the march the user watches; the support index
        // within it is almost always 0 and read as a frozen bar.
        progress(&coupling_context.phase("SOLVING", coupling_iteration, profile));
        let request_id = format!("solve:{event_id}:{coupling_iteration}:{support_iteration}");
        let active_state_id = format!("support-state:{support_iteration}");
        // WHICH ITERATION OF WHICH SEARCH, AND WHAT THE CONTACTS WERE DOING:
        // a refusal from inside the kernel says nothing about the state it
        // was handed, so the census travels with the message.
        let kernel_result = kernel
            .solve(&solved_frame, &request_id, &active_state_id)
            .map_err(|cause| {
                let friction_loads = solved_frame
                    .loads
                    .iter()
                    .filter(|load| load.load_id().starts_with("load:friction:"))
                    .count();
                PalletError::sentence(format!(
                    "{} [support search round {support_iteration}; supports {}; {friction_loads} friction loads applied]",
                    cause.message,
                    support_census(&support_state),
                ))
            })?;
        if kernel_result.diagnostics.iter().any(|entry| entry.severity == "ERROR") {
            let codes: Vec<&str> = kernel_result
                .diagnostics
                .iter()
                .map(|entry| entry.code.as_str())
                .collect();
            return Err(PalletError::sentence(format!("KERNEL_FAILURE:{}", codes.join(","))));
        }
        // Each ACTIVE support's latest measured tension, refreshed per round.
        for state in &support_state.active_state {
            if state.normal_state != "ACTIVE" {
                continue;
            }
            let tension = kernel_result
                .reactions
                .iter()
                .find(|candidate| candidate.support_id == state.support_id)
                .map(|reaction| (-reaction.force.y).max(0.0))
                .unwrap_or(0.0);
            pole_tension_n.insert(state.support_id.clone(), tension);
        }
        progress(&coupling_context.phase("RECOVERING", coupling_iteration, profile));
        let frozen_ids: HashSet<String> = bistable_frozen.keys().cloned().collect();
        let update: SupportUpdate =
            update_pallet_support_active_state(&support_state, &kernel_result, profile, &frozen_ids)?;
        last_changes = update.changes.clone();
        last_residual_n = update.complementarity_residual_n;
        // The two-cycle's detectable face: a LIFT proposed for the support
        // the immediately preceding pivot RESTORED — plus the widened
        // recurrence: a third toggle is a cycle, whatever came between.
        for change in &update.changes {
            if change.reason == "LIFT_OFF" || change.reason == "CONTACT_RESTORED" {
                *contact_toggle_count.entry(change.support_id.clone()).or_insert(0) += 1;
            }
        }
        let recurring_toggle = update.changes.first().is_some_and(|change| {
            (change.reason == "LIFT_OFF" || change.reason == "CONTACT_RESTORED")
                && contact_toggle_count.get(&change.support_id).copied().unwrap_or(0) >= 3
                && !bistable_frozen.contains_key(&change.support_id)
        });
        let exact_inverse = update.changes.len() == 1
            && previous_pivot.as_ref().is_some_and(|previous| {
                let change = &update.changes[0];
                change.support_id == previous.support_id
                    && change.reason == "LIFT_OFF"
                    && previous.reason == "CONTACT_RESTORED"
            });
        if exact_inverse || recurring_toggle {
            let change = &update.changes[0];
            let tension_n = kernel_result
                .reactions
                .iter()
                .find(|candidate| candidate.support_id == change.support_id)
                .map(|reaction| (-reaction.force.y).max(0.0))
                .unwrap_or(0.0);
            // The amplitude the admissibility judges is the CYCLE'S OWN moved
            // force: the contact's recorded ACTIVE-pole tension — exactly the
            // force the freeze will hold and disclose. The GLOBAL residual is
            // deliberately not in this test (it carries later-indexed
            // violators the least-index walk has not reached yet), and the
            // lifted pole's k·penetration reads phantom kilonewtons when a
            // lateral contact's friction fixing releases with it. Over-release
            // safety lives in the freeze acting at PRESENCE, not here.
            let amplitude_n = tension_n.max(
                pole_tension_n.get(&change.support_id).copied().unwrap_or(0.0),
            );
            if amplitude_n <= bistable_admissible_n {
                bistable_frozen.insert(change.support_id.clone(), tension_n);
                // The proposal is REJECTED, not applied: the state where the
                // support is ACTIVE stands, and the next round's update sees
                // the freeze. One extra solve per freeze, at most N.
                progress(&ProgressEmission::Note {
                    message: format!(
                        "[mechanics] bistable contact frozen: {} at {} N (admissible {} N)",
                        change.support_id,
                        to_precision(tension_n, 3),
                        to_precision(bistable_admissible_n, 3),
                    ),
                });
                previous_pivot = None;
                continue;
            }
        }
        previous_pivot = if update.changes.len() == 1 { Some(update.changes[0].clone()) } else { None };
        if update.requires_resolve {
            support_state = update.next;
            continue;
        }
        if !update.converged {
            return Err(PalletError::sentence("NON_CONVERGED: support active state residual"));
        }
        let pallet_response =
            recover_pallet_top_response(&solved_frame, &projection, &kernel_result, profile)?;
        // A DEFLECTION IS NOT A JOURNEY, AND NOT A PIROUETTE: nothing on a
        // pallet deflects by the width of the pallet, and a linear solve
        // reporting a radian has left the theory it was computed with,
        // whatever the residual says.
        // The pallet's own footprint IS the bound: TS takes overallMm × 0.001,
        // i.e. the dimension in metres — nothing on a pallet deflects by the
        // width of the pallet. The caller hands dimensions already in metres.
        let bound = pallet_overall_m.0.max(pallet_overall_m.1);
        let rotation_bound_rad = 0.1;
        let drifted = pallet_response.contacts.iter().find(|contact| {
            contact.translation.vec().hypot3() > bound
                || contact.rotation.vec().hypot3() > rotation_bound_rad
        });
        if let Some(drifted) = drifted {
            let worst_rotation = kernel_result
                .node_responses
                .iter()
                .max_by(|left, right| {
                    left.rotation
                        .vec()
                        .hypot3()
                        .total_cmp(&right.rotation.vec().hypot3())
                })
                .expect("node responses");
            let worst_translation = kernel_result
                .node_responses
                .iter()
                .max_by(|left, right| {
                    left.translation
                        .vec()
                        .hypot3()
                        .total_cmp(&right.translation.vec().hypot3())
                })
                .expect("node responses");
            let carried_n: f64 = partition
                .pallet_contacts
                .iter()
                .map(|contact| contact.force.y.abs())
                .sum();
            let distribution = if carried_n == 0.0 {
                "nothing carried".to_string()
            } else {
                partition
                    .pallet_contacts
                    .iter()
                    .map(|contact| format!("{:.3}", contact.force.y.abs() / carried_n))
                    .collect::<Vec<_>>()
                    .join("/")
            };
            return Err(PalletError::sentence(format!(
                "PALLET_RESPONSE_IMPLAUSIBLE at coupling round {coupling_iteration} carrying shares {distribution}: {} moved ({}, {}, {}) m against the pallet's own {:.4} m and turned ({}, {}, {}) rad against {rotation_bound_rad} rad of linear validity; supports {}; worst rotation {} ({}, {}, {}) rad; worst translation {} ({}, {}, {}) m; kernel says [{}]",
                drifted.contact_id,
                to_precision(drifted.translation.x, 4),
                to_precision(drifted.translation.y, 4),
                to_precision(drifted.translation.z, 4),
                bound,
                to_precision(drifted.rotation.x, 3),
                to_precision(drifted.rotation.y, 3),
                to_precision(drifted.rotation.z, 3),
                support_census(&support_state),
                worst_rotation.node_id,
                to_precision(worst_rotation.rotation.x, 3),
                to_precision(worst_rotation.rotation.y, 3),
                to_precision(worst_rotation.rotation.z, 3),
                worst_translation.node_id,
                to_precision(worst_translation.translation.x, 3),
                to_precision(worst_translation.translation.y, 3),
                to_precision(worst_translation.translation.z, 3),
                kernel_diagnostics_sentence(&kernel_result),
            )));
        }
        let mut bistable_contacts: Vec<(String, f64)> = bistable_frozen.into_iter().collect();
        bistable_contacts.sort_by(|left, right| compare_canonical_utf8(&left.0, &right.0));
        return Ok(SupportedSolve {
            support_state: update.next,
            solved_frame,
            kernel_result,
            pallet_response,
            bistable_contacts,
        });
    }
    let by_reason = {
        let mut reasons: Vec<&str> = last_changes.iter().map(|change| change.reason.as_str()).collect();
        reasons.sort_unstable();
        reasons.dedup();
        reasons
            .iter()
            .map(|reason| {
                let count = last_changes.iter().filter(|change| change.reason == *reason).count();
                format!("{reason}x{count}")
            })
            .collect::<Vec<_>>()
            .join(" ")
    };
    let distinct_supports: HashSet<&str> =
        last_changes.iter().map(|change| change.support_id.as_str()).collect();
    Err(PalletError::sentence(format!(
        "NON_CONVERGED: support active-state iteration limit ({} iterations; last round {} over {} supports; complementarity residual {} N; example {})",
        profile.coupled_iteration_limit,
        if by_reason.is_empty() { "no changes".to_string() } else { by_reason },
        distinct_supports.len(),
        to_precision(last_residual_n, 3),
        last_changes
            .first()
            .map(|change| change.support_id.clone())
            .unwrap_or_else(|| "none".into()),
    )))
}

/// The per-round residual context the progress emissions carry.
pub struct CouplingProgressContext {
    pub coupling_round: u32,
    pub share_residual: Option<f64>,
    pub translation_residual_m: Option<f64>,
    pub rotation_residual_rad: Option<f64>,
}

impl CouplingProgressContext {
    fn phase(
        &self,
        phase: &'static str,
        coupling_iteration: u32,
        profile: &NumericalAcceptanceProfile,
    ) -> ProgressEmission {
        ProgressEmission::Phase {
            phase,
            coupling_round: coupling_iteration,
            total: profile.coupled_iteration_limit,
            share_residual: self.share_residual,
            translation_residual_m: self.translation_residual_m,
            rotation_residual_rad: self.rotation_residual_rad,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct CouplingConvergence {
    pub converged: bool,
    pub reason: &'static str,
    pub maximum_translation_residual_m: Option<f64>,
    pub maximum_rotation_residual_rad: Option<f64>,
    pub maximum_load_share_residual: Option<f64>,
}

/// TS `evaluateCouplingConvergence` — three residuals against three
/// tolerances, canonical interface order on both sides.
pub fn evaluate_coupling_convergence(
    previous_response: Option<&PalletTopResponse>,
    current_response: &PalletTopResponse,
    previous_interfaces: &[crate::schema::UnitLoadInterface],
    current_interfaces: &[crate::schema::UnitLoadInterface],
    profile: &NumericalAcceptanceProfile,
) -> PalletResult<CouplingConvergence> {
    let canonical = |values: &[crate::schema::UnitLoadInterface]| {
        let mut sorted: Vec<crate::schema::UnitLoadInterface> = values.to_vec();
        sorted.sort_by(|left, right| compare_canonical_utf8(&left.interface_id, &right.interface_id));
        sorted
    };
    let previous_sorted = canonical(previous_interfaces);
    let current_sorted = canonical(current_interfaces);
    if previous_sorted.len() != current_sorted.len()
        || previous_sorted
            .iter()
            .zip(current_sorted.iter())
            .any(|(left, right)| left.interface_id != right.interface_id)
    {
        return Err(PalletError::sentence("COUPLING_INTERFACE_COVERAGE_MISMATCH"));
    }
    let Some(previous) = previous_response else {
        return Ok(CouplingConvergence {
            converged: false,
            reason: "INITIAL_RESPONSE",
            maximum_translation_residual_m: None,
            maximum_rotation_residual_rad: None,
            maximum_load_share_residual: None,
        });
    };
    let previous_contacts: HashMap<&str, &crate::schema::PalletTopContactResponse> = previous
        .contacts
        .iter()
        .map(|contact| (contact.contact_id.as_str(), contact))
        .collect();
    if previous.contacts.len() != current_response.contacts.len()
        || current_response
            .contacts
            .iter()
            .any(|contact| !previous_contacts.contains_key(contact.contact_id.as_str()))
    {
        return Err(PalletError::sentence("COUPLING_CONTACT_COVERAGE_MISMATCH"));
    }
    let mut maximum_translation_residual_m: f64 = 0.0;
    let mut maximum_rotation_residual_rad: f64 = 0.0;
    for contact in &current_response.contacts {
        let prior = previous_contacts[contact.contact_id.as_str()];
        maximum_translation_residual_m = maximum_translation_residual_m.max(
            Vec3 {
                x: contact.translation.x - prior.translation.x,
                y: contact.translation.y - prior.translation.y,
                z: contact.translation.z - prior.translation.z,
            }
            .hypot3(),
        );
        maximum_rotation_residual_rad = maximum_rotation_residual_rad.max(
            Vec3 {
                x: contact.rotation.x - prior.rotation.x,
                y: contact.rotation.y - prior.rotation.y,
                z: contact.rotation.z - prior.rotation.z,
            }
            .hypot3(),
        );
    }
    let maximum_load_share_residual = current_sorted
        .iter()
        .zip(previous_sorted.iter())
        .fold(0.0_f64, |maximum, (current, previous)| {
            maximum.max((current.load_share_ratio - previous.load_share_ratio).abs())
        });
    let converged = maximum_translation_residual_m <= profile.coupled_translation_tolerance_m
        && maximum_rotation_residual_rad <= profile.coupled_rotation_tolerance_rad
        && maximum_load_share_residual <= profile.coupled_load_share_tolerance;
    Ok(CouplingConvergence {
        converged,
        reason: if converged { "TOLERANCES_SATISFIED" } else { "RESIDUAL_EXCEEDS_TOLERANCE" },
        maximum_translation_residual_m: Some(maximum_translation_residual_m),
        maximum_rotation_residual_rad: Some(maximum_rotation_residual_rad),
        maximum_load_share_residual: Some(maximum_load_share_residual),
    })
}

#[derive(Debug, Clone, serde::Serialize)]
#[serde(rename_all = "camelCase")]
pub struct CoupledEventResult {
    pub support_state: CompiledPalletSupportState,
    pub solved_frame: AnalysisFrame,
    pub kernel_result: KernelResult,
    pub frame_response: FrameResponseRecovery,
    pub pallet_response: PalletTopResponse,
    pub unit_load_state: UnitLoadActiveState,
    pub bistable_contacts: Vec<BistableDisclosure>,
    pub coupling_rounds: u32,
}

#[derive(Debug, Clone, serde::Serialize)]
#[serde(rename_all = "camelCase")]
pub struct BistableDisclosure {
    pub support_id: String,
    #[serde(rename = "residualN")]
    pub residual_n: f64,
}

/// TS `solveEvent`'s coupled loop — partition → supported solve → advance →
/// convergence, seeding each next search re-stuck (SLIP IS ONE-WAY WITHIN A
/// SEARCH, and between searches every surviving contact grips where the
/// converged solve left it).
pub fn solve_coupled_event(
    kernel: &mut dyn KernelPort,
    base_frame: &AnalysisFrame,
    member_map: &PalletMemberMap,
    initial_support_state: &CompiledPalletSupportState,
    initial_unit_state: &UnitLoadActiveState,
    pallet_overall_m: (f64, f64),
    profile: &NumericalAcceptanceProfile,
    event_id: &str,
    progress: ProgressSink<'_>,
) -> PalletResult<CoupledEventResult> {
    let mut support_state = initial_support_state.clone();
    let mut state = initial_unit_state.clone();
    // The last thing the coupling loop was still arguing about, plus WHAT THE
    // LOOP WAS DOING: the share residual of every round, in order, says at a
    // glance whether a walk is closing in slowly or going round in a circle.
    let mut last_coupling: Option<CouplingConvergence> = None;
    let mut share_residual_trace: Vec<String> = Vec::new();
    let mut context = CouplingProgressContext {
        coupling_round: 0,
        share_residual: None,
        translation_residual_m: None,
        rotation_residual_rad: None,
    };
    for coupling_iteration in 0..profile.coupled_iteration_limit {
        context.coupling_round = coupling_iteration;
        let supported = solve_supported_frame(
            kernel,
            base_frame,
            member_map,
            &support_state,
            &state,
            pallet_overall_m,
            profile,
            event_id,
            coupling_iteration,
            &context,
            progress,
        )?;
        let next = advance_unit_load_partition(&state, &supported.pallet_response, profile)?;
        let convergence = evaluate_coupling_convergence(
            state.pallet_response.as_ref(),
            &supported.pallet_response,
            &state.interfaces,
            &next.interfaces,
            profile,
        )?;
        // The NEXT search's seed, not this round's answer: surviving contacts
        // grip where the converged solve left them.
        support_state =
            restick_converged_pallet_support_state(&supported.support_state, &supported.kernel_result)?;
        state = next;
        last_coupling = Some(convergence);
        context.share_residual = convergence.maximum_load_share_residual;
        context.translation_residual_m = convergence.maximum_translation_residual_m;
        context.rotation_residual_rad = convergence.maximum_rotation_residual_rad;
        if let Some(residual) = convergence.maximum_load_share_residual {
            let carrying = state
                .interfaces
                .iter()
                .filter(|entry| entry.lower_package_instance_id.is_none() && entry.load_share_ratio > 0.0)
                .count();
            share_residual_trace.push(format!("{}/{carrying}", to_precision(residual, 2)));
        }
        if !convergence.converged {
            continue;
        }
        // Terminal-only station recovery: inside the loop the recovered
        // stations feed nothing — the advance consumes the TOP response, not
        // the beam stations — so recovering per round was pure discarded
        // output. The recovery itself is unchanged; only the rounds that
        // threw it away are gone.
        let frame_response = kernel.recover(&supported.solved_frame, &supported.kernel_result, 3)?;
        return Ok(CoupledEventResult {
            support_state: supported.support_state,
            solved_frame: supported.solved_frame,
            kernel_result: supported.kernel_result,
            frame_response,
            pallet_response: supported.pallet_response,
            unit_load_state: state,
            bistable_contacts: supported
                .bistable_contacts
                .into_iter()
                .map(|(support_id, residual_n)| BistableDisclosure { support_id, residual_n })
                .collect(),
            coupling_rounds: coupling_iteration + 1,
        });
    }
    let against = |residual: Option<f64>, tolerance: f64, unit: &str| -> String {
        match residual {
            None => "n/a".into(),
            Some(residual) => format!(
                "{} {unit} (x{} of {tolerance})",
                to_precision(residual, 3),
                to_precision(residual / tolerance, 3),
            ),
        }
    };
    Err(PalletError::sentence(format!(
        "NON_CONVERGED: pallet/unit-load coupled iteration limit ({} iterations; last round {}; translation {}; rotation {}; load share {}; share residual/contacts carrying by round {})",
        profile.coupled_iteration_limit,
        last_coupling.map(|coupling| coupling.reason).unwrap_or("never evaluated"),
        against(
            last_coupling.and_then(|coupling| coupling.maximum_translation_residual_m),
            profile.coupled_translation_tolerance_m,
            "m",
        ),
        against(
            last_coupling.and_then(|coupling| coupling.maximum_rotation_residual_rad),
            profile.coupled_rotation_tolerance_rad,
            "rad",
        ),
        against(
            last_coupling.and_then(|coupling| coupling.maximum_load_share_residual),
            profile.coupled_load_share_tolerance,
            "",
        ),
        share_residual_trace.join(" "),
    )))
}
