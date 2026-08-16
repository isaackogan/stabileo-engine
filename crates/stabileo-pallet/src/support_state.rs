//! ONE STEP of the contact search, and the asymmetry in it is deliberate.
//!
//! Ported literally from the application's `support-active-state.ts`; the
//! essays travel with the functions they explain (PORTING.md rule 2).
//!
//! A SUPPORT IS A SPRING NOW, not a level. It carries `force = stiffness ×
//! penetration` as an OUTPUT of the solve, so there is no prescribed
//! settlement to guess, no fixed point iterating towards one, and nothing
//! left to damp. What the search still does is the one thing the kernel's
//! spring cannot: the kernel spring is LINEAR AND BILATERAL, and a floor is
//! not. Unilaterality therefore lives here, in which supports are PRESENT.
//!
//! SLIP IS A FORCE NOW, not an absence. A slipping contact keeps its
//! tangential freedoms released — a sliding foot is not held anywhere — and
//! the floor applies the force it can still supply, μN, in the direction
//! that broke it.
//!
//! SLIP is still one-way WITHIN a search: a released tangential DOF reports
//! no shear because it is released, so re-sticking on that zero is the
//! search believing its own decision back.
//!
//! LIFT-OFF is two-way, because its re-entry condition is not a reaction at
//! all — it is PENETRATION, which a released node reports honestly precisely
//! because nothing is holding it up.
//!
//! NO ANTI-CYCLING BUDGET LIVES HERE, and one was tried: capping restorations
//! re-creates one-way lift-off. A floor cannot be five-sixths removed.

use std::collections::{HashMap, HashSet};

use crate::schema::{
    CompiledPalletSupportState, FrameLoad, FrameSupport, KernelResult, NumericalAcceptanceProfile,
    PlainXZ, SupportContactActiveStateEntry, Tagged3,
};
use crate::types::{PalletError, PalletResult, Vec3};

#[derive(Debug, Clone, PartialEq)]
pub struct ActiveStateChange {
    pub support_id: String,
    pub contact_id: String,
    pub previous_normal_state: String,
    pub next_normal_state: String,
    pub previous_tangential_state: String,
    pub next_tangential_state: String,
    pub reason: String,
}

#[derive(Debug, Clone)]
pub struct SupportUpdate {
    pub next: CompiledPalletSupportState,
    pub changes: Vec<ActiveStateChange>,
    pub complementarity_residual_n: f64,
    /// The worst residual tension among bistable-frozen supports — the
    /// disclosed price of expressing a boundary contact in a discrete active
    /// set. Zero when nothing is frozen. Reported beside, never inside,
    /// `complementarity_residual_n`.
    pub bistable_residual_n: f64,
    pub converged: bool,
    pub requires_resolve: bool,
}

struct Evaluation {
    kept: SupportContactActiveStateEntry,
    proposed: SupportContactActiveStateEntry,
    change: Option<ActiveStateChange>,
}

pub fn update_pallet_support_active_state(
    support_state: &CompiledPalletSupportState,
    kernel_result: &KernelResult,
    profile: &NumericalAcceptanceProfile,
    bistable_frozen_support_ids: &HashSet<String>,
) -> PalletResult<SupportUpdate> {
    let mut bistable_residual_n: f64 = 0.0;
    let reactions: HashMap<&str, &crate::schema::KernelReaction> = kernel_result
        .reactions
        .iter()
        .map(|reaction| (reaction.support_id.as_str(), reaction))
        .collect();
    let mut complementarity_residual_n: f64 = 0.0;
    // Where every node ended up, for the half of the contact condition a
    // released support can still be measured against. A lifted-off support
    // reports NO reaction — that is what releasing it means — but it can
    // still read the DISPLACEMENT, and the floor has not moved.
    let node_translations: HashMap<&str, Vec3> = kernel_result
        .node_responses
        .iter()
        .map(|node| (node.node_id.as_str(), node.translation.vec()))
        .collect();
    let support_node_ids: HashMap<&str, &str> = support_state
        .frame_supports
        .iter()
        .map(|support| (support.support_id.as_str(), support.node_id.as_str()))
        .collect();

    let mut evaluations: Vec<Evaluation> = Vec::with_capacity(support_state.active_state.len());
    for state in &support_state.active_state {
        let reaction = reactions.get(state.support_id.as_str()).copied();
        let normal_reaction_n = reaction
            .map(|entry| entry.force.y)
            .unwrap_or(state.vertical_reaction_n);
        let frozen = bistable_frozen_support_ids.contains(&state.support_id);
        if frozen {
            bistable_residual_n = bistable_residual_n.max((-normal_reaction_n).max(0.0));
        } else {
            complementarity_residual_n = complementarity_residual_n.max((-normal_reaction_n).max(0.0));
        }
        let contact_for_state = support_state
            .bearing_contacts
            .iter()
            .find(|candidate| candidate.contact_id == state.contact_id)
            .ok_or_else(|| PalletError::sentence(format!("BEARING_CONTACT_MISSING:{}", state.contact_id)))?;
        // CONTACT IS TWO-WAY. A released node that has sunk BELOW the floor
        // it was released from is the other complementarity condition
        // failing, and it is measurable precisely because it is released:
        // nothing is holding it up, so its displacement is the truth. It
        // comes back.
        let penetration_m = {
            let translation = support_node_ids
                .get(state.support_id.as_str())
                .and_then(|node_id| node_translations.get(node_id));
            match translation {
                None => 0.0,
                Some(translation) => {
                    let floor_m = -contact_for_state.mechanics.settlement.value;
                    (floor_m - translation.y).max(0.0)
                }
            }
        };
        let restored = state.normal_state == "LIFTED_OFF" && penetration_m > profile.length_tolerance_m;
        // A frozen support IS active — that is what the freeze means —
        // whatever its tension reads this round; the tension was routed to
        // `bistable_residual_n` above.
        let remains_active = frozen
            || restored
            || (state.normal_state == "ACTIVE"
                && normal_reaction_n >= -profile.complementarity_tolerance_n);
        let tangential_reaction_n = match reaction {
            None => state.tangential_reaction_n,
            Some(entry) => PlainXZ { x: entry.force.x, z: entry.force.z },
        };
        let tangential_demand_n = tangential_reaction_n.x.hypot(tangential_reaction_n.z);
        let friction_capacity_n =
            normal_reaction_n.max(0.0) * contact_for_state.mechanics.friction_coefficient.value;
        let measured_utilization = if friction_capacity_n > 0.0 {
            tangential_demand_n / friction_capacity_n
        } else if tangential_demand_n == 0.0 {
            0.0
        } else {
            f64::MAX
        };
        // SLIP IS ONE-WAY WITHIN A SEARCH, exactly as lift-off already is: a
        // released tangential DOF reports no shear because it is released,
        // and the search must not believe its own decision back.
        let next_tangential_state = if !remains_active {
            "INACTIVE"
        } else if state.tangential_state == "SLIP" {
            "SLIP"
        } else if measured_utilization > 1.0 + profile.coupled_load_share_tolerance {
            "SLIP"
        } else {
            "STICK"
        };
        // A released support KEEPS the utilization that released it: the
        // recorded number is the demand that overcame its capacity, which is
        // the honest answer to "why is this contact sliding".
        let friction_utilization = if state.tangential_state == "SLIP" && next_tangential_state == "SLIP" {
            state.friction_utilization
        } else {
            Some(measured_utilization)
        };
        // THE DIRECTION THAT BROKE IT, kept for the same reason: Coulomb
        // needs a direction, and a slipping contact's own reaction reads
        // zero through released freedoms.
        let held_tangential_reaction_n = if state.tangential_state == "SLIP" && next_tangential_state == "SLIP" {
            state.tangential_reaction_n
        } else {
            tangential_reaction_n
        };
        let change: Option<ActiveStateChange> = if state.normal_state == "ACTIVE" && !remains_active {
            Some(ActiveStateChange {
                support_id: state.support_id.clone(),
                contact_id: state.contact_id.clone(),
                previous_normal_state: state.normal_state.clone(),
                next_normal_state: "LIFTED_OFF".into(),
                previous_tangential_state: state.tangential_state.clone(),
                next_tangential_state: "INACTIVE".into(),
                reason: "LIFT_OFF".into(),
            })
        } else if state.normal_state == "LIFTED_OFF" && restored {
            Some(ActiveStateChange {
                support_id: state.support_id.clone(),
                contact_id: state.contact_id.clone(),
                previous_normal_state: state.normal_state.clone(),
                next_normal_state: "ACTIVE".into(),
                previous_tangential_state: state.tangential_state.clone(),
                next_tangential_state: next_tangential_state.into(),
                reason: "CONTACT_RESTORED".into(),
            })
        } else if remains_active && state.tangential_state != next_tangential_state {
            // STICK to SLIP is the only tangential change a search can make:
            // slip is one-way, and INACTIVE belongs to a lifted support,
            // which `remains_active` has already excluded.
            Some(ActiveStateChange {
                support_id: state.support_id.clone(),
                contact_id: state.contact_id.clone(),
                previous_normal_state: state.normal_state.clone(),
                next_normal_state: "ACTIVE".into(),
                previous_tangential_state: state.tangential_state.clone(),
                next_tangential_state: next_tangential_state.into(),
                reason: "STICK_TO_SLIP".into(),
            })
        } else {
            None
        };
        // The state with this round's transition APPLIED.
        let proposed = SupportContactActiveStateEntry {
            support_id: state.support_id.clone(),
            contact_id: state.contact_id.clone(),
            normal_state: if remains_active { "ACTIVE".into() } else { "LIFTED_OFF".into() },
            // A RESTORED CONTACT RE-STICKS WHERE IT TOUCHED DOWN. The stick
            // anchor was compiled once, at the support's birth position, and
            // never moved — so a contact released during a lateral event,
            // whose node has since gone sideways with the elastic frame, was
            // prescribed back to its HISTORICAL position on restore. Static
            // friction cannot pull a block across the floor to where it used
            // to stand; it grips the contact where it lands.
            tangential_displacement_m: if !restored {
                state.tangential_displacement_m
            } else {
                match support_node_ids
                    .get(state.support_id.as_str())
                    .and_then(|node_id| node_translations.get(node_id))
                {
                    None => state.tangential_displacement_m,
                    Some(translation) => PlainXZ { x: translation.x, z: translation.z },
                }
            },
            // MEASURED, not iterated towards: the spring's compression is how
            // far its node actually went, which the solve has just reported.
            vertical_reaction_n: normal_reaction_n.max(0.0),
            vertical_compression_m: if remains_active { penetration_m } else { 0.0 },
            tangential_state: if remains_active { next_tangential_state.into() } else { "INACTIVE".into() },
            tangential_reaction_n: if remains_active {
                held_tangential_reaction_n
            } else {
                PlainXZ { x: 0.0, z: 0.0 }
            },
            friction_utilization: if remains_active { friction_utilization } else { None },
        };
        // The state with its transition DENIED: previous active-set states
        // kept, this solve's measurements recorded against them.
        let kept = if state.normal_state == "ACTIVE" {
            SupportContactActiveStateEntry {
                support_id: state.support_id.clone(),
                contact_id: state.contact_id.clone(),
                normal_state: state.normal_state.clone(),
                vertical_reaction_n: normal_reaction_n.max(0.0),
                vertical_compression_m: penetration_m,
                tangential_state: state.tangential_state.clone(),
                tangential_displacement_m: state.tangential_displacement_m,
                tangential_reaction_n: if state.tangential_state == "SLIP" {
                    state.tangential_reaction_n
                } else {
                    tangential_reaction_n
                },
                friction_utilization: if state.tangential_state == "SLIP" {
                    state.friction_utilization
                } else {
                    Some(measured_utilization)
                },
            }
        } else {
            state.clone()
        };
        evaluations.push(Evaluation { kept, proposed, change });
    }

    // ONE PIVOT PER SOLVE — the first violator in the state's own canonical
    // order, every other proposal denied until the next solve. Applying every
    // proposed change at once is a Jacobi step, and a Jacobi step on coupled
    // contacts CYCLES. One change per solve is the classical single-pivot
    // active-set step, and taking the FIRST violator in canonical order is
    // Murty's least-index rule — deterministic and hash-stable.
    let pivot_index = evaluations.iter().position(|evaluation| evaluation.change.is_some());
    let active_state: Vec<SupportContactActiveStateEntry> = evaluations
        .iter()
        .enumerate()
        .map(|(index, evaluation)| {
            if Some(index) == pivot_index { evaluation.proposed.clone() } else { evaluation.kept.clone() }
        })
        .collect();
    let mut changes: Vec<ActiveStateChange> = Vec::new();
    if let Some(index) = pivot_index {
        changes.push(evaluations[index].change.clone().expect("pivot change"));
    }
    let next = CompiledPalletSupportState {
        frame_supports: compose_frame_supports(&support_state.frame_supports, &active_state)?,
        active_state,
        // Application identity, re-derived app-side on the terminal state.
        active_state_sha256: "internal".into(),
        ..support_state.clone()
    };
    let converged = changes.is_empty() && complementarity_residual_n <= profile.complementarity_tolerance_n;
    let requires_resolve = !changes.is_empty();
    Ok(SupportUpdate {
        next,
        changes,
        complementarity_residual_n,
        bistable_residual_n,
        converged,
        requires_resolve,
    })
}

/// A SLIPPING CONTACT IS A LOAD, not a silence: its freedoms are released and
/// the floor's remaining μN is applied at the same node, in the direction the
/// contact was resisting when it broke.
pub fn pallet_slip_friction_loads(support_state: &CompiledPalletSupportState) -> Vec<FrameLoad> {
    let node_by_support: HashMap<&str, &str> = support_state
        .frame_supports
        .iter()
        .map(|support| (support.support_id.as_str(), support.node_id.as_str()))
        .collect();
    support_state
        .active_state
        .iter()
        .filter(|state| state.normal_state == "ACTIVE" && state.tangential_state == "SLIP")
        .filter_map(|state| {
            let node_id = node_by_support.get(state.support_id.as_str())?;
            let contact = support_state
                .bearing_contacts
                .iter()
                .find(|candidate| candidate.contact_id == state.contact_id)?;
            let magnitude = state.tangential_reaction_n.x.hypot(state.tangential_reaction_n.z);
            if !(magnitude > 0.0) {
                return None;
            }
            let friction_n = state.vertical_reaction_n * contact.mechanics.friction_coefficient.value;
            if !(friction_n > 0.0) {
                return None;
            }
            Some(FrameLoad::NodalForce {
                load_id: format!("load:friction:{}", state.support_id),
                node_id: (*node_id).into(),
                force: Tagged3::polar(
                    "N",
                    Vec3 {
                        x: (state.tangential_reaction_n.x / magnitude) * friction_n,
                        y: 0.0,
                        z: (state.tangential_reaction_n.z / magnitude) * friction_n,
                    },
                ),
                application: None,
            })
        })
        .collect()
}

/// The kernel-facing support rows a given active state implies — shared by
/// the per-round update and the between-searches re-stick so the two can
/// never disagree about what STICK or ACTIVE means to the kernel.
pub fn compose_frame_supports(
    frame_supports: &[FrameSupport],
    active_state: &[SupportContactActiveStateEntry],
) -> PalletResult<Vec<FrameSupport>> {
    frame_supports
        .iter()
        .map(|support| {
            let state = active_state
                .iter()
                .find(|candidate| candidate.support_id == support.support_id)
                .ok_or_else(|| {
                    PalletError::sentence(format!("SUPPORT_STATE_MISSING:{}", support.support_id))
                })?;
            let active = state.normal_state == "ACTIVE";
            let sticking = active && state.tangential_state == "STICK";
            // A support that reaches this search is elastic on its vertical
            // axis, and the search does not invent one that is not: an
            // unsupported vertical DOF with no spring under it would reach
            // the kernel as a singular model rather than as a sentence.
            let elastic = support.elastic_stiffness.as_ref();
            if elastic.map(|stiffness| stiffness.y <= 0.0).unwrap_or(true) {
                return Err(PalletError::sentence(format!(
                    "SUPPORT_NOT_ELASTIC:{}",
                    support.support_id
                )));
            }
            Ok(FrameSupport {
                active,
                // The VERTICAL flag stays false whether the support is
                // present or not. Its stiffness is what holds the node, and
                // the kernel reports no reaction on an axis that is both
                // fixed and elastic.
                fixed_dofs: [sticking, false, sticking, false, false, false],
                prescribed_translations: if active {
                    Some(Tagged3::polar(
                        "m",
                        Vec3 {
                            x: if sticking { state.tangential_displacement_m.x } else { 0.0 },
                            y: 0.0,
                            z: if sticking { state.tangential_displacement_m.z } else { 0.0 },
                        },
                    ))
                } else {
                    None
                },
                ..support.clone()
            })
        })
        .collect()
}

/// SLIP IS ONE-WAY WITHIN A SEARCH — and this is where "within" is enforced.
/// Between searches the honest seed is: every surviving contact GRIPS WHERE
/// THE CONVERGED SOLVE LEFT IT. Lift-offs carry over untouched — normal-state
/// warm starting is what makes the coupled loop's searches short — and the
/// terminal state of each search is reported exactly as solved; only the SEED
/// of the following search is re-stuck.
pub fn restick_converged_pallet_support_state(
    support_state: &CompiledPalletSupportState,
    kernel_result: &KernelResult,
) -> PalletResult<CompiledPalletSupportState> {
    let node_translations: HashMap<&str, Vec3> = kernel_result
        .node_responses
        .iter()
        .map(|node| (node.node_id.as_str(), node.translation.vec()))
        .collect();
    let support_node_ids: HashMap<&str, &str> = support_state
        .frame_supports
        .iter()
        .map(|support| (support.support_id.as_str(), support.node_id.as_str()))
        .collect();
    let active_state: Vec<SupportContactActiveStateEntry> = support_state
        .active_state
        .iter()
        .map(|state| {
            if state.normal_state != "ACTIVE" || state.tangential_state != "SLIP" {
                return state.clone();
            }
            let translation = support_node_ids
                .get(state.support_id.as_str())
                .and_then(|node_id| node_translations.get(node_id));
            match translation {
                None => state.clone(),
                Some(translation) => SupportContactActiveStateEntry {
                    tangential_state: "STICK".into(),
                    tangential_displacement_m: PlainXZ { x: translation.x, z: translation.z },
                    tangential_reaction_n: PlainXZ { x: 0.0, z: 0.0 },
                    friction_utilization: None,
                    ..state.clone()
                },
            }
        })
        .collect();
    Ok(CompiledPalletSupportState {
        frame_supports: compose_frame_supports(&support_state.frame_supports, &active_state)?,
        active_state,
        active_state_sha256: "internal".into(),
        ..support_state.clone()
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::schema::{
        BearingContactMechanics, KernelNodeResponse, KernelReaction, PalletBearingContact, Quantity,
    };
    use serde_json::Map;

    fn quantity(unit: &str, value: f64) -> Quantity {
        Quantity { unit: unit.into(), value }
    }

    fn profile() -> NumericalAcceptanceProfile {
        NumericalAcceptanceProfile {
            schema_version: "FP_NUMERICAL_ACCEPTANCE_PROFILE_1".into(),
            profile_id: "test".into(),
            profile_sha256: "internal".into(),
            geometry_tolerance_m: 1e-9,
            force_tolerance_n: 1e-3,
            moment_tolerance_nm: 1e-3,
            length_tolerance_m: 1e-8,
            complementarity_tolerance_n: 1e-6,
            coupled_iteration_limit: 200,
            coupled_translation_tolerance_m: 3e-5,
            coupled_rotation_tolerance_rad: 3e-5,
            coupled_load_share_tolerance: 1e-3,
        }
    }

    fn support(support_id: &str, node_id: &str) -> FrameSupport {
        FrameSupport {
            support_id: support_id.into(),
            node_id: node_id.into(),
            active: true,
            fixed_dofs: [true, false, true, false, false, false],
            prescribed_translations: Some(Tagged3::polar("m", Vec3::ZERO)),
            prescribed_rotations: None,
            elastic_stiffness: Some(Tagged3::polar("N_per_m", Vec3 { x: 0.0, y: 1.0e6, z: 0.0 })),
        }
    }

    fn contact(contact_id: &str, support_id: &str, node_id: &str, mu: f64, settlement_m: f64) -> PalletBearingContact {
        PalletBearingContact {
            contact_id: contact_id.into(),
            support_id: support_id.into(),
            node_id: node_id.into(),
            mechanics: BearingContactMechanics {
                vertical_stiffness: quantity("N_per_m", 1.0e6),
                bearing_capacity: quantity("N", 1.0e5),
                friction_coefficient: quantity("ratio", mu),
                settlement: quantity("m", settlement_m),
                extra: Map::new(),
            },
            extra: Map::new(),
        }
    }

    fn entry(support_id: &str, contact_id: &str, normal: &str, tangential: &str) -> SupportContactActiveStateEntry {
        SupportContactActiveStateEntry {
            support_id: support_id.into(),
            contact_id: contact_id.into(),
            normal_state: normal.into(),
            vertical_compression_m: 0.0,
            vertical_reaction_n: 0.0,
            tangential_state: tangential.into(),
            tangential_displacement_m: PlainXZ { x: 0.0, z: 0.0 },
            tangential_reaction_n: PlainXZ { x: 0.0, z: 0.0 },
            friction_utilization: if tangential == "STICK" { Some(0.0) } else { None },
        }
    }

    fn state_of(entries: Vec<SupportContactActiveStateEntry>, supports: Vec<FrameSupport>, contacts: Vec<PalletBearingContact>) -> CompiledPalletSupportState {
        CompiledPalletSupportState {
            schema_version: "FP_COMPILED_PALLET_SUPPORT_STATE_2".into(),
            condition_id: "condition:floor".into(),
            frame_supports: supports,
            bearing_contacts: contacts,
            active_state: entries,
            active_state_sha256: "internal".into(),
            extra: Map::new(),
        }
    }

    fn kernel_result(reactions: Vec<KernelReaction>, nodes: Vec<KernelNodeResponse>) -> KernelResult {
        KernelResult {
            schema_version: "FP_KERNEL_RESULT_1".into(),
            request_id: "r".into(),
            active_state_id: "a".into(),
            active_state_hash: "internal".into(),
            node_responses: nodes,
            reactions,
            element_end_forces: vec![],
            connector_responses: vec![],
            constraint_forces: vec![],
            applied_resultant: zero_resultant(),
            reaction_resultant: zero_resultant(),
            force_residual: Tagged3::polar("N", Vec3::ZERO),
            moment_residual: Tagged3::axial("N_m", Vec3::ZERO),
            diagnostics: vec![],
            result_hash: "internal".into(),
        }
    }

    fn zero_resultant() -> crate::schema::Resultant {
        crate::schema::Resultant {
            force: crate::schema::ResultantAxes {
                x: quantity("N", 0.0),
                y: quantity("N", 0.0),
                z: quantity("N", 0.0),
            },
            moment: crate::schema::ResultantAxes {
                x: quantity("N_m", 0.0),
                y: quantity("N_m", 0.0),
                z: quantity("N_m", 0.0),
            },
        }
    }

    fn reaction(support_id: &str, force: Vec3) -> KernelReaction {
        KernelReaction {
            support_id: support_id.into(),
            force: Tagged3::polar("N", force),
            moment: Tagged3::axial("N_m", Vec3::ZERO),
        }
    }

    fn node(node_id: &str, translation: Vec3) -> KernelNodeResponse {
        KernelNodeResponse {
            node_id: node_id.into(),
            translation: Tagged3::polar("m", translation),
            rotation: Tagged3::axial("rad", Vec3::ZERO),
        }
    }

    #[test]
    fn a_tension_support_lifts_off_as_the_single_pivot() {
        let state = state_of(
            vec![entry("s:a", "c:a", "ACTIVE", "STICK"), entry("s:b", "c:b", "ACTIVE", "STICK")],
            vec![support("s:a", "n:a"), support("s:b", "n:b")],
            vec![contact("c:a", "s:a", "n:a", 0.5, 0.0), contact("c:b", "s:b", "n:b", 0.5, 0.0)],
        );
        // BOTH supports in tension; only the FIRST pivots (Murty least-index).
        let result = kernel_result(
            vec![reaction("s:a", Vec3 { x: 0.0, y: -5.0, z: 0.0 }), reaction("s:b", Vec3 { x: 0.0, y: -3.0, z: 0.0 })],
            vec![node("n:a", Vec3::ZERO), node("n:b", Vec3::ZERO)],
        );
        let update = update_pallet_support_active_state(&state, &result, &profile(), &HashSet::new()).unwrap();
        assert_eq!(update.changes.len(), 1);
        assert_eq!(update.changes[0].support_id, "s:a");
        assert_eq!(update.changes[0].reason, "LIFT_OFF");
        assert_eq!(update.next.active_state[0].normal_state, "LIFTED_OFF");
        assert_eq!(update.next.active_state[1].normal_state, "ACTIVE");
        assert!((update.complementarity_residual_n - 5.0).abs() < 1e-12);
        assert!(update.requires_resolve);
        assert!(!update.converged);
        // The lifted support's kernel row releases everything.
        assert!(!update.next.frame_supports[0].active);
        assert!(update.next.frame_supports[0].prescribed_translations.is_none());
    }

    #[test]
    fn a_sunk_released_node_restores_and_resticks_where_it_landed() {
        let mut lifted = entry("s:a", "c:a", "LIFTED_OFF", "INACTIVE");
        lifted.friction_utilization = None;
        let state = state_of(
            vec![lifted],
            vec![support("s:a", "n:a")],
            vec![contact("c:a", "s:a", "n:a", 0.5, 0.0)],
        );
        let result = kernel_result(
            vec![],
            vec![node("n:a", Vec3 { x: 0.002, y: -1.0e-6, z: -0.003 })],
        );
        let update = update_pallet_support_active_state(&state, &result, &profile(), &HashSet::new()).unwrap();
        assert_eq!(update.changes.len(), 1);
        assert_eq!(update.changes[0].reason, "CONTACT_RESTORED");
        let restored = &update.next.active_state[0];
        assert_eq!(restored.normal_state, "ACTIVE");
        assert_eq!(restored.tangential_state, "STICK");
        // Re-anchored at the landed lateral position, not the birth origin.
        assert!((restored.tangential_displacement_m.x - 0.002).abs() < 1e-15);
        assert!((restored.tangential_displacement_m.z + 0.003).abs() < 1e-15);
        let prescribed = update.next.frame_supports[0].prescribed_translations.as_ref().unwrap();
        assert!((prescribed.x - 0.002).abs() < 1e-15);
        assert!((prescribed.z + 0.003).abs() < 1e-15);
    }

    #[test]
    fn over_capacity_stick_slips_once_and_becomes_a_friction_load() {
        let state = state_of(
            vec![entry("s:a", "c:a", "ACTIVE", "STICK")],
            vec![support("s:a", "n:a")],
            vec![contact("c:a", "s:a", "n:a", 0.5, 0.0)],
        );
        // Normal 100 N, capacity 50 N, demand 60 N sideways → utilization 1.2.
        let result = kernel_result(
            vec![reaction("s:a", Vec3 { x: 60.0, y: 100.0, z: 0.0 })],
            vec![node("n:a", Vec3::ZERO)],
        );
        let update = update_pallet_support_active_state(&state, &result, &profile(), &HashSet::new()).unwrap();
        assert_eq!(update.changes[0].reason, "STICK_TO_SLIP");
        let slipping = &update.next.active_state[0];
        assert_eq!(slipping.tangential_state, "SLIP");
        assert!((slipping.friction_utilization.unwrap() - 1.2).abs() < 1e-12);
        // μN along the held direction: 100 × 0.5 along +x.
        let loads = pallet_slip_friction_loads(&update.next);
        assert_eq!(loads.len(), 1);
        match &loads[0] {
            FrameLoad::NodalForce { force, load_id, node_id, .. } => {
                assert_eq!(load_id, "load:friction:s:a");
                assert_eq!(node_id, "n:a");
                assert!((force.x - 50.0).abs() < 1e-12);
                assert_eq!(force.y, 0.0);
                assert_eq!(force.z, 0.0);
            }
            other => panic!("expected nodal force, got {other:?}"),
        }
    }

    #[test]
    fn converged_when_no_changes_and_residual_inside_tolerance() {
        let state = state_of(
            vec![entry("s:a", "c:a", "ACTIVE", "STICK")],
            vec![support("s:a", "n:a")],
            vec![contact("c:a", "s:a", "n:a", 0.5, 0.0)],
        );
        let result = kernel_result(
            vec![reaction("s:a", Vec3 { x: 1.0, y: 100.0, z: 0.0 })],
            vec![node("n:a", Vec3::ZERO)],
        );
        let update = update_pallet_support_active_state(&state, &result, &profile(), &HashSet::new()).unwrap();
        assert!(update.converged);
        assert!(update.changes.is_empty());
        assert_eq!(update.complementarity_residual_n, 0.0);
    }

    #[test]
    fn a_frozen_support_routes_tension_to_the_disclosure_and_stays() {
        let state = state_of(
            vec![entry("s:a", "c:a", "ACTIVE", "STICK")],
            vec![support("s:a", "n:a")],
            vec![contact("c:a", "s:a", "n:a", 0.5, 0.0)],
        );
        let result = kernel_result(
            vec![reaction("s:a", Vec3 { x: 0.0, y: -2.5, z: 0.0 })],
            vec![node("n:a", Vec3::ZERO)],
        );
        let frozen: HashSet<String> = ["s:a".to_string()].into_iter().collect();
        let update = update_pallet_support_active_state(&state, &result, &profile(), &frozen).unwrap();
        assert!(update.changes.is_empty());
        assert!(update.converged);
        assert!((update.bistable_residual_n - 2.5).abs() < 1e-12);
        assert_eq!(update.complementarity_residual_n, 0.0);
        assert_eq!(update.next.active_state[0].normal_state, "ACTIVE");
    }

    #[test]
    fn restick_reseeds_slip_as_stick_at_the_landed_position() {
        let mut slipping = entry("s:a", "c:a", "ACTIVE", "SLIP");
        slipping.tangential_reaction_n = PlainXZ { x: 10.0, z: 0.0 };
        slipping.friction_utilization = Some(1.4);
        slipping.vertical_reaction_n = 100.0;
        let state = state_of(
            vec![slipping, entry("s:b", "c:b", "LIFTED_OFF", "INACTIVE")],
            vec![support("s:a", "n:a"), support("s:b", "n:b")],
            vec![contact("c:a", "s:a", "n:a", 0.5, 0.0), contact("c:b", "s:b", "n:b", 0.5, 0.0)],
        );
        let result = kernel_result(
            vec![],
            vec![node("n:a", Vec3 { x: 0.0005, y: -1e-5, z: 0.0002 }), node("n:b", Vec3::ZERO)],
        );
        let reseeded = restick_converged_pallet_support_state(&state, &result).unwrap();
        assert_eq!(reseeded.active_state[0].tangential_state, "STICK");
        assert!((reseeded.active_state[0].tangential_displacement_m.x - 0.0005).abs() < 1e-15);
        assert_eq!(reseeded.active_state[0].friction_utilization, None);
        // Lift-offs carry over untouched.
        assert_eq!(reseeded.active_state[1].normal_state, "LIFTED_OFF");
        // The kernel rows re-fix the re-stuck support's tangential freedoms.
        assert_eq!(reseeded.frame_supports[0].fixed_dofs, [true, false, true, false, false, false]);
    }
}
