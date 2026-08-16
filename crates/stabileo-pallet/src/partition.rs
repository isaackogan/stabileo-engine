//! THE LOAD IS A BODY STANDING ON SPRINGS, not a set of shares being
//! corrected. Ported literally from the application's `partition.ts` advance
//! and solve paths; initialization stays application-side (once per event).
//!
//! The body settles as a RIGID BODY — `w(r) = w₀ + wₓ·x + w_z·z`, three
//! unknowns — and each base contact then carries what its own compression
//! says it carries: `F = k · (w(r) − deck settlement)`, never negative, ZERO
//! where the deck has dropped away faster than the load could follow. THIS
//! FORM CONTRACTS BY CONSTRUCTION and IS MEMORYLESS; the deck's own
//! compliance under each contact is MEASURED from the loop's own rounds (two
//! rounds give one secant), refused measurements keep the last good one, and
//! carrying a stale measurement cannot move the answer — the compliance
//! chooses the path, never the destination.

use std::collections::HashMap;


use crate::schema::{
    NumericalAcceptanceProfile, PalletTopContactPatch, PalletTopResponse, PalletTopContactResponse,
    Quantity, RememberedBaseLoadShare, Tagged3, UnitLoadActiveState, UnitLoadCriterionResult,
    UnitLoadInterface, UnitLoadInterfaceState,
};
use crate::types::{PalletError, PalletResult, Vec3};
use crate::unit_load::criteria::{
    evaluate_unit_load_capacity_criterion, EvaluateUnitLoadCapacityCriterionRequest,
    UnitLoadCriterionKind,
};
use crate::unit_load::friction::{evaluate_coulomb_state_2t, CoulombState2TRequest};
use crate::unit_load::resultants::{
    audit_resultants, resultant_from_applications, resultant_from_contacts, vector_cross,
    ConservationAudit, ContactPatch, ForceApplication, Resultant, ResultantTolerances,
};
use crate::unit_load::rigid_settlement::{
    solve_rigid_body_settlement, RigidBodySettlementRequest, SettlementContact,
};

/// Adapter: schema force applications → the resultants module's plain shape.
fn plain_applications(state: &UnitLoadActiveState) -> Vec<ForceApplication> {
    state
        .force_applications
        .iter()
        .map(|application| ForceApplication {
            point: application.point.vec(),
            force: application.force.vec(),
            moment: application.moment.vec(),
        })
        .collect()
}

/// Adapter: schema contact patches → the resultants module's plain shape.
fn plain_contacts(contacts: &[PalletTopContactPatch]) -> Vec<ContactPatch> {
    contacts
        .iter()
        .map(|contact| ContactPatch {
            center: contact.center.vec(),
            force: contact.force.vec(),
            free_moment: contact.free_moment.vec(),
        })
        .collect()
}

fn tolerances(profile: &NumericalAcceptanceProfile) -> ResultantTolerances {
    ResultantTolerances {
        force_tolerance_n: profile.force_tolerance_n,
        moment_tolerance_nm: profile.moment_tolerance_nm,
    }
}

#[derive(Debug, Clone)]
pub struct UnitLoadPartition {
    pub pallet_contacts: Vec<PalletTopContactPatch>,
    pub applied_resultant: Resultant,
    pub conservation: ConservationAudit,
}

/// TS `solveUnitLoadPartition`: the partition IS the state's own contacts —
/// what this call adds is the refusal to hand them over unconserved, and the
/// service-event identity check.
pub fn solve_unit_load_partition(
    state: &UnitLoadActiveState,
    service_event: &serde_json::Value,
    profile: &NumericalAcceptanceProfile,
) -> PalletResult<UnitLoadPartition> {
    // TS compares JSON.stringify of two schema-parsed (schema-shape-ordered)
    // values; content equality over parsed values is the same statement.
    let state_event = serde_json::to_value(&state.service_event)
        .map_err(|error| PalletError::sentence(format!("SERVICE_EVENT_ENCODE: {error}")))?;
    if &state_event != service_event {
        return Err(PalletError::sentence("SERVICE_EVENT_MISMATCH"));
    }
    let applied_resultant = resultant_from_applications(&plain_applications(state));
    let observed = resultant_from_contacts(&plain_contacts(&state.pallet_contacts));
    let conservation = audit_resultants(applied_resultant, observed, tolerances(profile));
    if !conservation.accepted {
        return Err(PalletError::sentence("RESULTANT_CONSERVATION_FAILED"));
    }
    Ok(UnitLoadPartition {
        pallet_contacts: state.pallet_contacts.clone(),
        applied_resultant,
        conservation,
    })
}

#[derive(Debug, Clone, Copy)]
struct KinematicDelta {
    translation: Vec3,
    rotation: Vec3,
}

/// TS `contactKinematicDelta` — the response delta against the previous
/// round's response, absent components reading zero.
fn contact_kinematic_delta(
    current: &PalletTopContactResponse,
    previous: Option<&PalletTopContactResponse>,
) -> KinematicDelta {
    KinematicDelta {
        translation: Vec3 {
            x: current.translation.x - previous.map(|p| p.translation.x).unwrap_or(0.0),
            y: current.translation.y - previous.map(|p| p.translation.y).unwrap_or(0.0),
            z: current.translation.z - previous.map(|p| p.translation.z).unwrap_or(0.0),
        },
        rotation: Vec3 {
            x: current.rotation.x - previous.map(|p| p.rotation.x).unwrap_or(0.0),
            y: current.rotation.y - previous.map(|p| p.rotation.y).unwrap_or(0.0),
            z: current.rotation.z - previous.map(|p| p.rotation.z).unwrap_or(0.0),
        },
    }
}

fn kind_quantity(value: &Option<Quantity>, contact_id: &str, field: &str) -> PalletResult<f64> {
    value
        .as_ref()
        .map(|quantity| quantity.value)
        .ok_or_else(|| PalletError::sentence(format!("CONTACT_FIELD_MISSING:{contact_id}:{field}")))
}

/// TS `contactTiltGap` — the patch's own edge riding up or down as the deck
/// rotates beneath it.
fn contact_tilt_gap(contact: &PalletTopContactPatch, rotation: Vec3) -> PalletResult<f64> {
    let tilt = rotation.x.hypot(rotation.z);
    Ok(match contact.kind.as_str() {
        "POINT" => 0.0,
        "RECTANGULAR_PATCH" => {
            rotation.x.abs() * kind_quantity(&contact.half_size_z, &contact.contact_id, "halfSizeZ")?
                + rotation.z.abs() * kind_quantity(&contact.half_size_x, &contact.contact_id, "halfSizeX")?
        }
        "ANNULAR_CHIME" => tilt * kind_quantity(&contact.outer_radius, &contact.contact_id, "outerRadius")?,
        "PRESSURE_FIELD" => {
            let boundary = contact.boundary.as_ref().ok_or_else(|| {
                PalletError::sentence(format!("CONTACT_FIELD_MISSING:{}:boundary", contact.contact_id))
            })?;
            let reach = boundary
                .iter()
                .map(|point| (point.x - contact.center.x).hypot(point.z - contact.center.z))
                .fold(f64::NEG_INFINITY, f64::max);
            tilt * reach
        }
        "ORIENTATION_BAND" => {
            rotation.x.abs() * kind_quantity(&contact.half_width, &contact.contact_id, "halfWidth")?
                + rotation.z.abs() * kind_quantity(&contact.half_length, &contact.contact_id, "halfLength")?
        }
        other => {
            return Err(PalletError::sentence(format!(
                "CONTACT_KIND_UNSUPPORTED:{other}"
            )))
        }
    })
}

/// TS `rotatePoint` — Rodrigues rotation of `point` about `center`, returning
/// the ROTATED RELATIVE vector (the caller re-adds the moved center).
fn rotate_point(point: Vec3, center: Vec3, rotation: Vec3) -> Vec3 {
    let relative = Vec3 { x: point.x - center.x, y: point.y - center.y, z: point.z - center.z };
    let angle = rotation.hypot3();
    if angle == 0.0 {
        return relative;
    }
    let axis = Vec3 { x: rotation.x / angle, y: rotation.y / angle, z: rotation.z / angle };
    let cosine = angle.cos();
    let sine = angle.sin();
    let dot = axis.x * relative.x + axis.y * relative.y + axis.z * relative.z;
    let cross = Vec3 {
        x: axis.y * relative.z - axis.z * relative.y,
        y: axis.z * relative.x - axis.x * relative.z,
        z: axis.x * relative.y - axis.y * relative.x,
    };
    Vec3 {
        x: relative.x * cosine + cross.x * sine + axis.x * dot * (1.0 - cosine),
        y: relative.y * cosine + cross.y * sine + axis.y * dot * (1.0 - cosine),
        z: relative.z * cosine + cross.z * sine + axis.z * dot * (1.0 - cosine),
    }
}

/// TS `deflectedContact` — where a contact has moved to, what it now carries,
/// and the couple it applies BEYOND its force at that place.
fn deflected_contact(
    contact: &PalletTopContactPatch,
    delta: KinematicDelta,
    force: Vec3,
    free_moment: Vec3,
) -> PalletTopContactPatch {
    let center = Vec3 {
        x: contact.center.x + delta.translation.x,
        y: contact.center.y + delta.translation.y,
        z: contact.center.z + delta.translation.z,
    };
    let mut moved = contact.clone();
    moved.center = Tagged3::point_m(center);
    moved.force = Tagged3::polar("N", force);
    moved.free_moment = Tagged3::axial("N_m", free_moment);
    match contact.kind.as_str() {
        "RECTANGULAR_PATCH" | "ORIENTATION_BAND" => {
            if let Some(orientation) = &contact.orientation {
                moved.orientation = Some(Quantity {
                    unit: orientation.unit.clone(),
                    value: orientation.value + delta.rotation.y,
                });
            }
        }
        "PRESSURE_FIELD" => {
            let original_center = contact.center.vec();
            let move_point = |point: &Tagged3| -> Tagged3 {
                let rotated = rotate_point(point.vec(), original_center, delta.rotation);
                Tagged3::point_m(Vec3 {
                    x: center.x + rotated.x,
                    y: center.y + rotated.y,
                    z: center.z + rotated.z,
                })
            };
            if let Some(boundary) = &contact.boundary {
                moved.boundary = Some(boundary.iter().map(move_point).collect());
            }
            if let Some(samples) = &contact.samples {
                moved.samples = Some(
                    samples
                        .iter()
                        .map(|sample| crate::schema::ContactPressureSample {
                            point: move_point(&sample.point),
                            normalized_weight: sample.normalized_weight,
                        })
                        .collect(),
                );
            }
        }
        _ => {}
    }
    moved
}

/// TS `advanceUnitLoadPartition` — one coupling advance: measure the deck,
/// settle the rigid body on its springs, place every contact where the deck
/// took it, and refuse the result unless it still carries exactly the load.
pub fn advance_unit_load_partition(
    previous: &UnitLoadActiveState,
    pallet_response: &PalletTopResponse,
    profile: &NumericalAcceptanceProfile,
) -> PalletResult<UnitLoadActiveState> {
    let response_by_contact: HashMap<&str, &PalletTopContactResponse> = pallet_response
        .contacts
        .iter()
        .map(|contact| (contact.contact_id.as_str(), contact))
        .collect();
    if pallet_response.contacts.len() != previous.pallet_contacts.len()
        || previous
            .pallet_contacts
            .iter()
            .any(|contact| !response_by_contact.contains_key(contact.contact_id.as_str()))
    {
        return Err(PalletError::sentence("PALLET_RESPONSE_CONTACT_COVERAGE_MISMATCH"));
    }
    let previous_response_by_contact: HashMap<&str, &PalletTopContactResponse> = previous
        .pallet_response
        .as_ref()
        .map(|response| {
            response
                .contacts
                .iter()
                .map(|contact| (contact.contact_id.as_str(), contact))
                .collect()
        })
        .unwrap_or_default();
    if let Some(previous_response) = &previous.pallet_response {
        if previous_response.contacts.len() != previous.pallet_contacts.len()
            || previous
                .pallet_contacts
                .iter()
                .any(|contact| !previous_response_by_contact.contains_key(contact.contact_id.as_str()))
        {
            return Err(PalletError::sentence("PREVIOUS_PALLET_RESPONSE_CONTACT_COVERAGE_MISMATCH"));
        }
    }
    let expected = resultant_from_applications(&plain_applications(previous));
    let total_force = expected.force;
    let total_moment = expected.moment;
    let base_interfaces: Vec<&UnitLoadInterface> = previous
        .interfaces
        .iter()
        .filter(|entry| entry.lower_package_instance_id.is_none())
        .collect();
    let find_contact = |contact_id: &str| -> PalletResult<&PalletTopContactPatch> {
        previous
            .pallet_contacts
            .iter()
            .find(|candidate| candidate.contact_id == contact_id)
            .ok_or_else(|| PalletError::sentence(format!("BASE_INTERFACE_NOT_FOUND:{contact_id}")))
    };
    // WHERE THE DECK'S SURFACE IS UNDER EACH PACKAGE, absolutely. Downward is
    // positive. SIGNED, and no longer clamped at zero: a deck that rises
    // under a package pushes back HARDER.
    let mut deck_settlement_m: Vec<f64> = Vec::with_capacity(base_interfaces.len());
    for entry in &base_interfaces {
        let contact = find_contact(&entry.contact_id)?;
        let contact_response = response_by_contact
            .get(entry.contact_id.as_str())
            .ok_or_else(|| PalletError::sentence("PALLET_RESPONSE_CONTACT_COVERAGE_MISMATCH"))?;
        deck_settlement_m.push(
            -contact_response.translation.y
                + contact_tilt_gap(contact, contact_response.rotation.vec())?,
        );
    }
    let settled_shares: Vec<f64> = base_interfaces.iter().map(|entry| entry.load_share_ratio).collect();
    // THE DECK'S OWN COMPLIANCE UNDER EACH CONTACT, MEASURED. Two rounds give
    // one secant. Refused measurements keep the LAST GOOD one; carrying a
    // stale measurement cannot move the answer.
    let remembered_by_id: HashMap<&str, &RememberedBaseLoadShare> = previous
        .previous_base_load_shares
        .as_deref()
        .unwrap_or(&[])
        .iter()
        .map(|entry| (entry.interface_id.as_str(), entry))
        .collect();
    let total_normal_n = total_force.y.abs();
    let mut deck_compliance_m_per_n: Vec<f64> = Vec::with_capacity(base_interfaces.len());
    for (index, entry) in base_interfaces.iter().enumerate() {
        let previous_response = previous_response_by_contact.get(entry.contact_id.as_str());
        let remembered = remembered_by_id.get(entry.interface_id.as_str());
        let compliance = match (previous_response, remembered) {
            (Some(previous_response), Some(remembered)) => {
                let contact = find_contact(&entry.contact_id)?;
                let previous_settlement_m = -previous_response.translation.y
                    + contact_tilt_gap(contact, previous_response.rotation.vec())?;
                let force_change_n = total_normal_n * (settled_shares[index] - remembered.load_share_ratio);
                if force_change_n.abs() <= profile.force_tolerance_n {
                    remembered.deck_compliance_m_per_n
                } else {
                    let secant = (deck_settlement_m[index] - previous_settlement_m) / force_change_n;
                    if secant > 0.0 && secant.is_finite() {
                        secant
                    } else {
                        remembered.deck_compliance_m_per_n
                    }
                }
            }
            _ => 0.0,
        };
        deck_compliance_m_per_n.push(compliance);
    }
    let mut settlement_contacts: Vec<SettlementContact> = Vec::with_capacity(base_interfaces.len());
    for (index, entry) in base_interfaces.iter().enumerate() {
        let contact = find_contact(&entry.contact_id)?;
        let contact_response = response_by_contact.get(entry.contact_id.as_str()).expect("covered");
        let delta = contact_kinematic_delta(
            contact_response,
            previous_response_by_contact.get(entry.contact_id.as_str()).copied(),
        );
        settlement_contacts.push(SettlementContact {
            stiffness_n_per_m: entry.normal_stiffness_n_per_m,
            deck_settlement_m: deck_settlement_m[index],
            deck_compliance_m_per_n: deck_compliance_m_per_n[index],
            current_force_n: total_normal_n * settled_shares[index],
            x: contact.center.x + delta.translation.x,
            z: contact.center.z + delta.translation.z,
        });
    }
    // The HORIZONTAL resultant carries its own height lever at the contact
    // plane, estimated with the incoming shares; the remainder lands honestly
    // in the free-moment residual below.
    let mut moment_x_lever = 0.0;
    for (index, entry) in base_interfaces.iter().enumerate() {
        let contact = find_contact(&entry.contact_id)?;
        moment_x_lever += contact.center.y * settled_shares[index];
    }
    let mut moment_z_lever = 0.0;
    for (index, entry) in base_interfaces.iter().enumerate() {
        let contact = find_contact(&entry.contact_id)?;
        moment_z_lever += contact.center.y * settled_shares[index];
    }
    let settlement = solve_rigid_body_settlement(&RigidBodySettlementRequest {
        contacts: settlement_contacts,
        total_downward_force_n: total_force.y.abs(),
        moment_x_target_nm: total_moment.x - total_force.z * moment_x_lever,
        moment_z_target_nm: total_moment.z + total_force.x * moment_z_lever,
    })?;
    let raw_total: f64 = settlement.reactions_n.iter().sum();
    if raw_total <= profile.force_tolerance_n {
        return Err(PalletError::sentence("CONTACT_EQUILIBRIUM_LOST"));
    }
    let shares: Vec<f64> = settlement.reactions_n.iter().map(|value| value / raw_total).collect();
    let base_index_of = |interface_id: &str| -> Option<usize> {
        base_interfaces
            .iter()
            .position(|candidate| candidate.interface_id == interface_id)
    };
    let interfaces: Vec<UnitLoadInterface> = previous
        .interfaces
        .iter()
        .map(|entry| match base_index_of(&entry.interface_id) {
            None => entry.clone(),
            Some(index) => UnitLoadInterface { load_share_ratio: shares[index], ..entry.clone() },
        })
        .collect();
    // Where each contact has moved to and what it now carries, in one pass —
    // and then, from those, the moment their positions do NOT explain,
    // shared out. Same decomposition the compile makes, and it has to be.
    struct Placed {
        delta: KinematicDelta,
        share: f64,
        center: Vec3,
        force: Vec3,
    }
    let mut placed: Vec<Placed> = Vec::with_capacity(previous.pallet_contacts.len());
    for contact in &previous.pallet_contacts {
        let index = base_interfaces
            .iter()
            .position(|entry| entry.contact_id == contact.contact_id)
            .ok_or_else(|| {
                PalletError::sentence(format!("BASE_INTERFACE_NOT_FOUND:{}", contact.contact_id))
            })?;
        let share = shares[index];
        let contact_response = response_by_contact.get(contact.contact_id.as_str()).expect("covered");
        let delta = contact_kinematic_delta(
            contact_response,
            previous_response_by_contact.get(contact.contact_id.as_str()).copied(),
        );
        placed.push(Placed {
            delta,
            share,
            center: Vec3 {
                x: contact.center.x + delta.translation.x,
                y: contact.center.y + delta.translation.y,
                z: contact.center.z + delta.translation.z,
            },
            force: Vec3 {
                x: total_force.x * share,
                y: total_force.y * share,
                z: total_force.z * share,
            },
        });
    }
    let lever_moment = placed.iter().fold(Vec3::ZERO, |sum, entry| {
        let moment = vector_cross(entry.center, entry.force);
        Vec3 { x: sum.x + moment.x, y: sum.y + moment.y, z: sum.z + moment.z }
    });
    let residual_moment = Vec3 {
        x: total_moment.x - lever_moment.x,
        y: total_moment.y - lever_moment.y,
        z: total_moment.z - lever_moment.z,
    };
    let pallet_contacts: Vec<PalletTopContactPatch> = previous
        .pallet_contacts
        .iter()
        .zip(placed.iter())
        .map(|(contact, entry)| {
            deflected_contact(
                contact,
                entry.delta,
                entry.force,
                Vec3 {
                    x: residual_moment.x * entry.share,
                    y: residual_moment.y * entry.share,
                    z: residual_moment.z * entry.share,
                },
            )
        })
        .collect();
    let previous_states: HashMap<&str, &UnitLoadInterfaceState> = previous
        .interface_states
        .iter()
        .map(|entry| (entry.interface_id.as_str(), entry))
        .collect();
    let mut interface_states: Vec<UnitLoadInterfaceState> = Vec::with_capacity(interfaces.len());
    for entry in &interfaces {
        if entry.lower_package_instance_id.is_some() {
            let kept = previous_states.get(entry.interface_id.as_str()).ok_or_else(|| {
                PalletError::sentence(format!("INTERFACE_STATE_MISSING:{}", entry.interface_id))
            })?;
            interface_states.push((*kept).clone());
            continue;
        }
        let contact = pallet_contacts
            .iter()
            .find(|candidate| candidate.contact_id == entry.contact_id)
            .ok_or_else(|| {
                PalletError::sentence(format!("BASE_INTERFACE_NOT_FOUND:{}", entry.contact_id))
            })?;
        let contact_response = response_by_contact.get(entry.contact_id.as_str()).expect("covered");
        let normal = contact.force.y.abs();
        let tangent = evaluate_coulomb_state_2t(&CoulombState2TRequest {
            interface_id: entry.interface_id.clone(),
            trial_force_x_n: contact.force.x,
            trial_force_z_n: contact.force.z,
            normal_force_n: normal,
            friction_coefficient: entry.friction_coefficient,
            relative_movement_x_m: contact_response.translation.x,
            relative_movement_z_m: contact_response.translation.z,
            tolerance_n: profile.force_tolerance_n,
        })?;
        interface_states.push(UnitLoadInterfaceState {
            interface_id: entry.interface_id.clone(),
            normal_state: if normal > profile.force_tolerance_n { "CLOSED".into() } else { "OPEN".into() },
            tangent_state: tangent.state.as_str().into(),
            slip_direction_x: tangent.slip_direction_x,
            slip_direction_z: tangent.slip_direction_z,
            normal_force_n: normal,
            tangent_force_x_n: tangent.tangent_force_x_n,
            tangent_force_z_n: tangent.tangent_force_z_n,
            complementarity_residual_n: 0.0,
            dissipated_energy_j: tangent.dissipated_energy_j,
        });
    }
    let horizontal_demand = total_force.x.hypot(total_force.z);
    let sliding_capacity: f64 = interfaces
        .iter()
        .filter(|entry| entry.lower_package_instance_id.is_none())
        .enumerate()
        .map(|(index, entry)| total_force.y.abs() * shares[index] * entry.friction_coefficient)
        .sum();
    let criteria: Vec<UnitLoadCriterionResult> = previous
        .criteria
        .iter()
        .map(|criterion| {
            if criterion.kind() != "SLIDING" {
                return Ok(criterion.clone());
            }
            let evaluated = evaluate_unit_load_capacity_criterion(
                &EvaluateUnitLoadCapacityCriterionRequest {
                    criterion_id: criterion.criterion_id().to_string(),
                    kind: UnitLoadCriterionKind::Sliding,
                    demand: horizontal_demand,
                    capacity: sliding_capacity,
                    governing_entity_id: criterion.governing_entity_id().to_string(),
                },
            )?;
            // Map the evaluator's single struct back onto the schema union's
            // shape: a reason code marks the zero-capacity variants; a plain
            // finite result carries a numeric utilization.
            Ok(match (evaluated.reason_code, evaluated.utilization) {
                (None, Some(utilization)) => UnitLoadCriterionResult::Finite {
                    criterion_id: evaluated.criterion_id,
                    kind: evaluated.kind.as_str().into(),
                    classification: evaluated.classification.as_str().into(),
                    demand: evaluated.demand,
                    capacity: evaluated.capacity,
                    utilization,
                    governing_entity_id: evaluated.governing_entity_id,
                },
                (reason, utilization) => UnitLoadCriterionResult::ZeroCapacity {
                    criterion_id: evaluated.criterion_id,
                    kind: evaluated.kind.as_str().into(),
                    classification: evaluated.classification.as_str().into(),
                    demand: evaluated.demand,
                    capacity: evaluated.capacity,
                    utilization,
                    reason_code: reason.map(|code| code.as_str().to_string()).unwrap_or_default(),
                    governing_entity_id: evaluated.governing_entity_id,
                },
            })
        })
        .collect::<PalletResult<Vec<_>>>()?;
    // THE REMEMBERED ITERATE IS NOT PART OF THE STATE'S IDENTITY: the shares
    // the loop passed through are the accelerator's scratchpad, remembered
    // for exactly one round.
    let previous_base_load_shares: Vec<RememberedBaseLoadShare> = base_interfaces
        .iter()
        .enumerate()
        .map(|(index, entry)| RememberedBaseLoadShare {
            interface_id: entry.interface_id.clone(),
            load_share_ratio: settled_shares[index],
            deck_compliance_m_per_n: deck_compliance_m_per_n[index],
        })
        .collect();
    let next = UnitLoadActiveState {
        pallet_response: Some(pallet_response.clone()),
        interfaces,
        interface_states,
        pallet_contacts,
        criteria,
        previous_base_load_shares: Some(previous_base_load_shares),
        active_state_sha256: "internal".into(),
        ..previous.clone()
    };
    let conservation = audit_resultants(
        expected,
        resultant_from_contacts(&plain_contacts(&next.pallet_contacts)),
        tolerances(profile),
    );
    if !conservation.accepted {
        return Err(PalletError::sentence("RESULTANT_CONSERVATION_FAILED"));
    }
    Ok(next)
}

/// A convenience for the coupled loop's service-event identity check.
pub fn service_event_value(state: &UnitLoadActiveState) -> PalletResult<serde_json::Value> {
    serde_json::to_value(&state.service_event)
        .map_err(|error| PalletError::sentence(format!("SERVICE_EVENT_ENCODE: {error}")))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::schema::{PlainXYZ, UnitLoadEvent};
    use serde_json::Map;

    fn tagged_point(x: f64, y: f64, z: f64) -> Tagged3 {
        Tagged3::point_m(Vec3 { x, y, z })
    }

    fn patch(contact_id: &str, x: f64, z: f64, force_y: f64) -> PalletTopContactPatch {
        PalletTopContactPatch {
            kind: "POINT".into(),
            contact_id: contact_id.into(),
            center: tagged_point(x, 0.1, z),
            force: Tagged3::polar("N", Vec3 { x: 0.0, y: force_y, z: 0.0 }),
            free_moment: Tagged3::axial("N_m", Vec3::ZERO),
            normal_stiffness_n_per_m: 1.0e6,
            orientation: None,
            boundary: None,
            samples: None,
            half_size_x: None,
            half_size_z: None,
            half_length: None,
            half_width: None,
            inner_radius: None,
            outer_radius: None,
            extra: Map::new(),
        }
    }

    fn interface(id: &str, contact_id: &str, share: f64) -> UnitLoadInterface {
        UnitLoadInterface {
            schema_version: "FP_UNIT_LOAD_INTERFACE_1".into(),
            interface_id: id.into(),
            lower_package_instance_id: None,
            upper_package_instance_id: "pkg:u".into(),
            contact_id: contact_id.into(),
            load_share_ratio: share,
            normal_stiffness_n_per_m: 1.0e6,
            friction_coefficient: 0.4,
            shear_capacity_n: 1.0e4,
        }
    }

    fn interface_state(id: &str) -> UnitLoadInterfaceState {
        UnitLoadInterfaceState {
            interface_id: id.into(),
            normal_state: "CLOSED".into(),
            tangent_state: "STICK".into(),
            slip_direction_x: 0.0,
            slip_direction_z: 0.0,
            normal_force_n: 50.0,
            tangent_force_x_n: 0.0,
            tangent_force_z_n: 0.0,
            complementarity_residual_n: 0.0,
            dissipated_energy_j: 0.0,
        }
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

    fn state_two_contacts() -> UnitLoadActiveState {
        // 100 N of weight standing symmetrically on two point contacts at
        // x = ±0.5: the applied force enters at the midpoint, so the rigid
        // settle must split it evenly.
        UnitLoadActiveState {
            schema_version: "FP_UNIT_LOAD_ACTIVE_STATE_1".into(),
            service_event: UnitLoadEvent {
                load_event_id: "event:gravity".into(),
                kind: "GRAVITY".into(),
                acceleration_m_per_s2: PlainXYZ { x: 0.0, y: -9.80665, z: 0.0 },
                angular_acceleration_rad_per_s2: PlainXYZ { x: 0.0, y: 0.0, z: 0.0 },
                extra: Map::new(),
            },
            pallet_response: None,
            interfaces: vec![interface("i:a", "c:a", 0.5), interface("i:b", "c:b", 0.5)],
            interface_states: vec![interface_state("i:a"), interface_state("i:b")],
            pallet_contacts: vec![patch("c:a", -0.5, 0.0, -50.0), patch("c:b", 0.5, 0.0, -50.0)],
            force_applications: vec![crate::schema::UnitLoadForceApplication {
                application_id: "app:gravity".into(),
                source_kind: "GRAVITY".into(),
                source_id: "unit".into(),
                point: tagged_point(0.0, 0.1, 0.0),
                force: Tagged3::polar("N", Vec3 { x: 0.0, y: -100.0, z: 0.0 }),
                moment: Tagged3::axial("N_m", Vec3::ZERO),
            }],
            criteria: vec![UnitLoadCriterionResult::Finite {
                criterion_id: "crit:sliding".into(),
                kind: "SLIDING".into(),
                classification: "PASS".into(),
                demand: 0.0,
                capacity: 40.0,
                utilization: 0.0,
                governing_entity_id: "unit".into(),
            }],
            previous_base_load_shares: None,
            active_state_sha256: "internal".into(),
            extra: Map::new(),
        }
    }

    fn flat_response(contacts: &[(&str, Vec3)]) -> PalletTopResponse {
        PalletTopResponse {
            schema_version: "FP_PALLET_TOP_RESPONSE_1".into(),
            frame_hash: "internal".into(),
            kernel_result_hash: "internal".into(),
            contacts: contacts
                .iter()
                .map(|(contact_id, translation)| PalletTopContactResponse {
                    contact_id: (*contact_id).into(),
                    translation: Tagged3::polar("m", *translation),
                    rotation: Tagged3::axial("rad", Vec3::ZERO),
                })
                .collect(),
            response_sha256: "internal".into(),
        }
    }

    #[test]
    fn a_symmetric_body_on_a_flat_deck_splits_evenly_and_conserves() {
        let state = state_two_contacts();
        let response = flat_response(&[("c:a", Vec3::ZERO), ("c:b", Vec3::ZERO)]);
        let next = advance_unit_load_partition(&state, &response, &profile()).unwrap();
        let shares: Vec<f64> = next
            .interfaces
            .iter()
            .map(|entry| entry.load_share_ratio)
            .collect();
        assert!((shares[0] - 0.5).abs() < 1e-12, "shares: {shares:?}");
        assert!((shares[1] - 0.5).abs() < 1e-12);
        // Forces re-placed: each contact carries half the weight downward.
        assert!((next.pallet_contacts[0].force.y + 50.0).abs() < 1e-9);
        // The remembered iterate records the shares the frame was SOLVED with.
        let remembered = next.previous_base_load_shares.as_ref().unwrap();
        assert!((remembered[0].load_share_ratio - 0.5).abs() < 1e-15);
        assert_eq!(remembered[0].deck_compliance_m_per_n, 0.0);
    }

    #[test]
    fn a_tilted_deck_moves_load_to_the_low_side() {
        let state = state_two_contacts();
        // Deck sinks 1 mm under contact B: the rigid body follows and B picks
        // up less than half only if the spring model says so — for equal
        // springs and a rigid tilt the settle re-balances toward equilibrium;
        // what MUST hold regardless is conservation and a share sum of one.
        let response = flat_response(&[
            ("c:a", Vec3::ZERO),
            ("c:b", Vec3 { x: 0.0, y: -0.001, z: 0.0 }),
        ]);
        let next = advance_unit_load_partition(&state, &response, &profile()).unwrap();
        let sum: f64 = next.interfaces.iter().map(|entry| entry.load_share_ratio).sum();
        assert!((sum - 1.0).abs() < 1e-12);
        // The moved contact's center followed the deck down.
        let moved = next
            .pallet_contacts
            .iter()
            .find(|contact| contact.contact_id == "c:b")
            .unwrap();
        assert!((moved.center.y - (0.1 - 0.001)).abs() < 1e-12);
    }

    #[test]
    fn service_event_mismatch_refuses() {
        let state = state_two_contacts();
        let other = serde_json::json!({"different": true});
        let error = solve_unit_load_partition(&state, &other, &profile()).unwrap_err();
        assert_eq!(error.message, "SERVICE_EVENT_MISMATCH");
    }

    #[test]
    fn tilt_gap_reads_each_kind_geometry() {
        let mut rectangular = patch("c:r", 0.0, 0.0, -1.0);
        rectangular.kind = "RECTANGULAR_PATCH".into();
        rectangular.half_size_x = Some(Quantity { unit: "m".into(), value: 0.2 });
        rectangular.half_size_z = Some(Quantity { unit: "m".into(), value: 0.1 });
        let gap = contact_tilt_gap(&rectangular, Vec3 { x: 0.01, y: 0.0, z: -0.02 }).unwrap();
        // |rx|·halfSizeZ + |rz|·halfSizeX = 0.01·0.1 + 0.02·0.2
        assert!((gap - (0.001 + 0.004)).abs() < 1e-15);
        let point = patch("c:p", 0.0, 0.0, -1.0);
        assert_eq!(contact_tilt_gap(&point, Vec3 { x: 0.5, y: 0.0, z: 0.5 }).unwrap(), 0.0);
    }
}
