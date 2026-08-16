//! Literal port of `packages/analysis/stabileo/src/normalize.ts`: the kernel's
//! raw 3D results become the application's `KernelResult`, with the
//! equilibrium audit standing between the two as the CORRUPT_OUTPUT guard.

use std::collections::HashMap;

use dedaliano_engine::types::{AnalysisResults3D, ConstraintForce as RawConstraintForce};
use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::kernel_bridge::compile::{CompiledStabileoModel, ParsedFrameConstraint};
use crate::kernel_bridge::coordinates::{from_stabileo_axial, from_stabileo_polar, StabileoLocalTriad};
use crate::kernel_bridge::diagnostics::{
    diagnostic_code_name, normalize_structured_diagnostics, severity_name,
};
use crate::kernel_bridge::equilibrium::audit_equilibrium;
use crate::kernel_bridge::id_map::bytewise_utf8_compare;
use crate::kernel_bridge::number_format::to_precision;
use crate::kernel_bridge::units::{sdk_force_value_to_solver, sdk_moment_value_to_solver};
use crate::schema::{
    KernelElementEndForces, KernelNodeResponse, KernelReaction, KernelResult, Quantity, Resultant,
    ResultantAxes, Tagged3,
};
use crate::types::{PalletError, PalletResult, Vec3};

fn corrupt<T>(message: String) -> PalletResult<T> {
    Err(PalletError::sentence(format!("CORRUPT_OUTPUT: {message}")))
}

fn finite(value: f64, label: &str) -> PalletResult<f64> {
    if !value.is_finite() {
        return corrupt(format!("non-finite {label}"));
    }
    Ok(if value == 0.0 { 0.0 } else { value })
}

fn add(a: Vec3, b: Vec3) -> Vec3 {
    Vec3 { x: a.x + b.x, y: a.y + b.y, z: a.z + b.z }
}

fn subtract(a: Vec3, b: Vec3) -> Vec3 {
    Vec3 { x: a.x - b.x, y: a.y - b.y, z: a.z - b.z }
}

fn scale(a: Vec3, factor: f64) -> Vec3 {
    Vec3 { x: a.x * factor, y: a.y * factor, z: a.z * factor }
}

fn dot(a: Vec3, b: Vec3) -> f64 {
    a.x * b.x + a.y * b.y + a.z * b.z
}

fn cross(a: Vec3, b: Vec3) -> Vec3 {
    Vec3 { x: a.y * b.z - a.z * b.y, y: a.z * b.x - a.x * b.z, z: a.x * b.y - a.y * b.x }
}

fn norm(a: Vec3) -> f64 {
    a.hypot3()
}

fn normalize_vector(a: Vec3) -> Vec3 {
    let length = norm(a);
    if length <= 1e-15 {
        return Vec3 { x: 1.0, y: 0.0, z: 0.0 };
    }
    scale(a, 1.0 / length)
}

fn combine(x: Vec3, y: Vec3, z: Vec3, components: Vec3) -> Vec3 {
    add(add(scale(x, components.x), scale(y, components.y)), scale(z, components.z))
}

fn require_unique_complete(label: &str, actual: &[usize], expected: &[usize]) -> PalletResult<()> {
    let seen: std::collections::HashSet<usize> = actual.iter().copied().collect();
    let expected_set: std::collections::HashSet<usize> = expected.iter().copied().collect();
    if seen.len() != actual.len() {
        return corrupt(format!("duplicate {label} output"));
    }
    if let Some(unknown) = actual.iter().find(|id| !expected_set.contains(id)) {
        return corrupt(format!("unknown numeric {label} ID {unknown}"));
    }
    if actual.len() != expected.len() {
        return corrupt(format!("{label} coverage does not match the compiled model"));
    }
    Ok(())
}

fn normalize_nodes(
    compiled: &CompiledStabileoModel,
    raw: &AnalysisResults3D,
) -> PalletResult<Vec<KernelNodeResponse>> {
    let expected: Vec<usize> = compiled.ids.nodes.entries().iter().map(|(_, id)| *id).collect();
    let actual: Vec<usize> = raw.displacements.iter().map(|item| item.node_id).collect();
    require_unique_complete("node", &actual, &expected)?;
    let mut responses = Vec::with_capacity(raw.displacements.len());
    for item in &raw.displacements {
        responses.push(KernelNodeResponse {
            node_id: compiled.ids.nodes.stable(item.node_id)?.to_string(),
            translation: from_stabileo_polar(
                Vec3 {
                    x: finite(item.ux, "node translation")?,
                    y: finite(item.uy, "node translation")?,
                    z: finite(item.uz, "node translation")?,
                },
                "m",
            ),
            rotation: from_stabileo_axial(
                Vec3 {
                    x: finite(item.rx, "node rotation")?,
                    y: finite(item.ry, "node rotation")?,
                    z: finite(item.rz, "node rotation")?,
                },
                "rad",
            ),
        });
    }
    responses.sort_by(|a, b| bytewise_utf8_compare(&a.node_id, &b.node_id));
    Ok(responses)
}

fn normalize_reactions(
    compiled: &CompiledStabileoModel,
    raw: &AnalysisResults3D,
) -> PalletResult<Vec<KernelReaction>> {
    let active_supports: Vec<&crate::schema::FrameSupport> =
        compiled.frame.supports.iter().filter(|support| support.active).collect();
    // A JS `Map` keyed by node: a later support at the same node replaces the
    // earlier one but keeps its key position.
    let mut support_node_keys: Vec<usize> = Vec::with_capacity(active_supports.len());
    let mut support_by_node: HashMap<usize, &crate::schema::FrameSupport> = HashMap::new();
    for support in &active_supports {
        let node = compiled.ids.nodes.numeric(&support.node_id)?;
        if support_by_node.insert(node, support).is_none() {
            support_node_keys.push(node);
        }
    }
    let actual: Vec<usize> = raw.reactions.iter().map(|item| item.node_id).collect();
    require_unique_complete("reaction", &actual, &support_node_keys)?;
    let mut reactions = Vec::with_capacity(raw.reactions.len());
    for item in &raw.reactions {
        let Some(support) = support_by_node.get(&item.node_id) else {
            return corrupt(format!("unknown numeric reaction node {}", item.node_id));
        };
        reactions.push(KernelReaction {
            support_id: support.support_id.clone(),
            force: from_stabileo_polar(
                Vec3 {
                    x: sdk_force_value_to_solver(finite(item.fx, "reaction force")?)?,
                    y: sdk_force_value_to_solver(finite(item.fy, "reaction force")?)?,
                    z: sdk_force_value_to_solver(finite(item.fz, "reaction force")?)?,
                },
                "N",
            ),
            moment: from_stabileo_axial(
                Vec3 {
                    x: sdk_moment_value_to_solver(finite(item.mx, "reaction moment")?)?,
                    y: sdk_moment_value_to_solver(finite(item.my, "reaction moment")?)?,
                    z: sdk_moment_value_to_solver(finite(item.mz, "reaction moment")?)?,
                },
                "N_m",
            ),
        });
    }
    reactions.sort_by(|a, b| bytewise_utf8_compare(&a.support_id, &b.support_id));
    Ok(reactions)
}

fn normalize_element_forces(
    compiled: &CompiledStabileoModel,
    raw: &AnalysisResults3D,
) -> PalletResult<Vec<KernelElementEndForces>> {
    let expected: Vec<usize> = compiled.ids.elements.entries().iter().map(|(_, id)| *id).collect();
    let actual: Vec<usize> = raw.element_forces.iter().map(|item| item.element_id).collect();
    require_unique_complete("element", &actual, &expected)?;
    let mut forces = Vec::with_capacity(raw.element_forces.len());
    for item in &raw.element_forces {
        let element_id = match compiled.ids.elements.stable(item.element_id) {
            Ok(element_id) => element_id.to_string(),
            Err(_) => return corrupt(format!("unknown numeric element ID {}", item.element_id)),
        };
        // TS `compiled.elementTriads.get(elementId)!` — the triad map is built
        // over the same element list the id map is, so this cannot miss.
        let Some(triad) = compiled.element_triads.get(&element_id) else {
            return corrupt(format!("element {element_id} has no local triad"));
        };
        let force = |x: f64, y: f64, z: f64| -> PalletResult<Tagged3> {
            Ok(from_stabileo_polar(
                combine(
                    triad.x,
                    triad.y,
                    triad.z,
                    Vec3 {
                        x: sdk_force_value_to_solver(finite(x, "element force")?)?,
                        y: sdk_force_value_to_solver(finite(y, "element force")?)?,
                        z: sdk_force_value_to_solver(finite(z, "element force")?)?,
                    },
                ),
                "N",
            ))
        };
        let moment = |x: f64, y: f64, z: f64| -> PalletResult<Tagged3> {
            Ok(from_stabileo_axial(
                combine(
                    triad.x,
                    triad.y,
                    triad.z,
                    Vec3 {
                        x: sdk_moment_value_to_solver(finite(x, "element moment")?)?,
                        y: sdk_moment_value_to_solver(finite(y, "element moment")?)?,
                        z: sdk_moment_value_to_solver(finite(z, "element moment")?)?,
                    },
                ),
                "N_m",
            ))
        };
        finite(item.length, "element length")?;
        forces.push(KernelElementEndForces {
            element_id: element_id.clone(),
            start_force: force(item.n_start, item.vy_start, item.vz_start)?,
            start_moment: moment(item.mx_start, item.my_start, item.mz_start)?,
            end_force: force(item.n_end, item.vy_end, item.vz_end)?,
            end_moment: moment(item.mx_end, item.my_end, item.mz_end)?,
        });
    }
    forces.sort_by(|a, b| bytewise_utf8_compare(&a.element_id, &b.element_id));
    Ok(forces)
}

fn connector_triad(node_i: Vec3, node_j: Vec3) -> StabileoLocalTriad {
    let x = normalize_vector(subtract(node_j, node_i));
    let reference = if x.x.abs() < 0.9 {
        Vec3 { x: 1.0, y: 0.0, z: 0.0 }
    } else {
        Vec3 { x: 0.0, y: 1.0, z: 0.0 }
    };
    let y = normalize_vector(cross(reference, x));
    StabileoLocalTriad { x, y, z: cross(x, y) }
}

/// The app's `ConnectorResponse`. `KernelResult.connector_responses` is a
/// `Vec<Value>` in `schema.rs` (the loop carries it opaque), so the typed shape
/// lives here and serializes into it.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ConnectorResponse {
    pub connector_id: String,
    pub relative_translation: Tagged3,
    pub relative_rotation: Tagged3,
    pub force_on_node_i: Tagged3,
    pub moment_on_node_i: Tagged3,
}

fn connector_responses(
    compiled: &CompiledStabileoModel,
    raw: &AnalysisResults3D,
) -> PalletResult<Vec<ConnectorResponse>> {
    let displacements: HashMap<usize, &dedaliano_engine::types::Displacement3D> =
        raw.displacements.iter().map(|item| (item.node_id, item)).collect();
    let mut responses = Vec::with_capacity(compiled.frame.connectors.len());
    for connector in &compiled.frame.connectors {
        let i_id = compiled.ids.nodes.numeric(&connector.node_i)?;
        let j_id = compiled.ids.nodes.numeric(&connector.node_j)?;
        let Some(i) = displacements.get(&i_id) else {
            return corrupt(format!(
                "connector {} has incomplete node I",
                connector.connector_id
            ));
        };
        let Some(j) = displacements.get(&j_id) else {
            return corrupt(format!(
                "connector {} has incomplete node J",
                connector.connector_id
            ));
        };
        let ui = Vec3 {
            x: finite(i.ux, "connector translation")?,
            y: finite(i.uy, "connector translation")?,
            z: finite(i.uz, "connector translation")?,
        };
        let uj = Vec3 {
            x: finite(j.ux, "connector translation")?,
            y: finite(j.uy, "connector translation")?,
            z: finite(j.uz, "connector translation")?,
        };
        let ri = Vec3 {
            x: finite(i.rx, "connector rotation")?,
            y: finite(i.ry, "connector rotation")?,
            z: finite(i.rz, "connector rotation")?,
        };
        let rj = Vec3 {
            x: finite(j.rx, "connector rotation")?,
            y: finite(j.ry, "connector rotation")?,
            z: finite(j.rz, "connector rotation")?,
        };
        let relative_translation = subtract(uj, ui);
        let relative_rotation = subtract(rj, ri);
        let node_i = compiled.input.nodes.get(&i_id.to_string()).map(|node| Vec3 {
            x: node.x,
            y: node.y,
            z: node.z,
        });
        let node_j = compiled.input.nodes.get(&j_id.to_string()).map(|node| Vec3 {
            x: node.x,
            y: node.y,
            z: node.z,
        });
        // TS `compiled.input.nodes[String(iId)]!` — the compiled node map covers
        // every frame node, so this cannot miss either.
        let (Some(node_i), Some(node_j)) = (node_i, node_j) else {
            return corrupt(format!(
                "connector {} names a node outside the compiled model",
                connector.connector_id
            ));
        };
        let triad = connector_triad(node_i, node_j);
        let local_translation = Vec3 {
            x: dot(relative_translation, triad.x),
            y: dot(relative_translation, triad.y),
            z: dot(relative_translation, triad.z),
        };
        let local_rotation = Vec3 {
            x: dot(relative_rotation, triad.x),
            y: dot(relative_rotation, triad.y),
            z: dot(relative_rotation, triad.z),
        };
        let value = |quantity: &Option<Quantity>| -> f64 {
            quantity.as_ref().map(|quantity| quantity.value).unwrap_or(0.0)
        };
        let local_force = Vec3 {
            x: -value(&connector.axial_stiffness) * local_translation.x,
            y: -value(&connector.shear_y_stiffness) * local_translation.y,
            z: -value(&connector.shear_z_stiffness) * local_translation.z,
        };
        let local_moment = Vec3 {
            x: -value(&connector.torsion_stiffness) * local_rotation.x,
            y: -value(&connector.bend_y_stiffness) * local_rotation.y,
            z: -value(&connector.bend_z_stiffness) * local_rotation.z,
        };
        responses.push(ConnectorResponse {
            connector_id: connector.connector_id.clone(),
            relative_translation: from_stabileo_polar(relative_translation, "m"),
            relative_rotation: from_stabileo_axial(relative_rotation, "rad"),
            force_on_node_i: from_stabileo_polar(
                combine(triad.x, triad.y, triad.z, local_force),
                "N",
            ),
            moment_on_node_i: from_stabileo_axial(
                combine(triad.x, triad.y, triad.z, local_moment),
                "N_m",
            ),
        });
    }
    responses.sort_by(|a, b| bytewise_utf8_compare(&a.connector_id, &b.connector_id));
    Ok(responses)
}

fn constraint_dofs(constraint: &ParsedFrameConstraint) -> Vec<String> {
    match constraint {
        ParsedFrameConstraint::RigidLink { dofs, .. }
        | ParsedFrameConstraint::EqualDof { dofs, .. } => dofs.clone(),
        ParsedFrameConstraint::LinearMpc { terms, .. } => {
            terms.iter().map(|term| format!("{}:{}", term.node_id, term.dof)).collect()
        }
        ParsedFrameConstraint::EccentricConnection { releases, .. } => {
            // TS `Object.entries(constraint.releases)` in the schema's own key
            // order, keeping the ones that are NOT released, upper-cased.
            [
                ("tx", releases.tx),
                ("ty", releases.ty),
                ("tz", releases.tz),
                ("rx", releases.rx),
                ("ry", releases.ry),
                ("rz", releases.rz),
            ]
            .iter()
            .filter(|(_, released)| !*released)
            .map(|(dof, _)| dof.to_uppercase())
            .collect()
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ConstraintForceTerm {
    pub kind: String,
    pub node_id: String,
    pub dof: String,
    pub value: Quantity,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ConstraintForceGroup {
    pub attribution: String,
    pub constraint_id: Option<String>,
    pub terms: Vec<ConstraintForceTerm>,
}

fn normalize_constraint_forces(
    compiled: &CompiledStabileoModel,
    raw_forces: &[RawConstraintForce],
) -> PalletResult<Vec<ConstraintForceGroup>> {
    let constraints = &compiled.parsed_constraints;
    // A JS `Map`: insertion-ordered, so a `Vec` of pairs with a linear probe.
    let mut groups: Vec<(String, ConstraintForceGroup)> = Vec::new();
    for item in raw_forces {
        let node_id = match compiled.ids.nodes.stable(item.node_id) {
            Ok(node_id) => node_id.to_string(),
            Err(_) => {
                return corrupt(format!("unknown numeric constraint-force node {}", item.node_id))
            }
        };
        let mapping: Option<(&str, f64, &str, &str)> = match item.dof.as_str() {
            "ux" => Some(("TX", 1.0, "N", "TRANSLATIONAL_FORCE")),
            "uy" => Some(("TZ", -1.0, "N", "TRANSLATIONAL_FORCE")),
            "uz" => Some(("TY", 1.0, "N", "TRANSLATIONAL_FORCE")),
            "rx" => Some(("RX", 1.0, "N_m", "ROTATIONAL_FORCE")),
            "ry" => Some(("RZ", -1.0, "N_m", "ROTATIONAL_FORCE")),
            "rz" => Some(("RY", 1.0, "N_m", "ROTATIONAL_FORCE")),
            _ => None,
        };
        let Some((dof, sign, unit, kind)) = mapping else {
            return corrupt(format!("unknown constraint force DOF {}", item.dof));
        };
        let candidates: Vec<&ParsedFrameConstraint> = constraints
            .iter()
            .filter(|constraint| match constraint {
                ParsedFrameConstraint::LinearMpc { .. } => {
                    constraint_dofs(constraint).contains(&format!("{node_id}:{dof}"))
                }
                ParsedFrameConstraint::RigidLink { master_node_id, slave_node_id, .. }
                | ParsedFrameConstraint::EqualDof { master_node_id, slave_node_id, .. }
                | ParsedFrameConstraint::EccentricConnection {
                    master_node_id,
                    slave_node_id,
                    ..
                } => {
                    (*master_node_id == node_id || *slave_node_id == node_id)
                        && constraint_dofs(constraint).contains(&dof.to_string())
                }
            })
            .collect();
        if candidates.is_empty() {
            return corrupt(format!("unattributable constraint force for {node_id}:{dof}"));
        }
        let constraint_id = if candidates.len() == 1 {
            Some(candidates[0].constraint_id().to_string())
        } else {
            None
        };
        let key = constraint_id.clone().unwrap_or_else(|| "\u{ffff}:AGGREGATED_AMBIGUOUS".to_string());
        let raw_value = finite(item.force, "constraint force")? * sign;
        let term = ConstraintForceTerm {
            kind: kind.to_string(),
            node_id: node_id.clone(),
            dof: dof.to_string(),
            value: Quantity {
                unit: unit.to_string(),
                value: if unit == "N" {
                    sdk_force_value_to_solver(raw_value)?
                } else {
                    sdk_moment_value_to_solver(raw_value)?
                },
            },
        };
        match groups.iter_mut().find(|(existing, _)| *existing == key) {
            Some((_, group)) => group.terms.push(term),
            None => groups.push((
                key,
                ConstraintForceGroup {
                    attribution: if constraint_id.is_none() {
                        "AGGREGATED_AMBIGUOUS".to_string()
                    } else {
                        "UNIQUE".to_string()
                    },
                    constraint_id,
                    terms: vec![term],
                },
            )),
        }
    }
    groups.sort_by(|(left, _), (right, _)| bytewise_utf8_compare(left, right));
    Ok(groups
        .into_iter()
        .map(|(_, mut group)| {
            group.terms.sort_by(|left, right| {
                bytewise_utf8_compare(
                    &format!("{}:{}", left.node_id, left.dof),
                    &format!("{}:{}", right.node_id, right.dof),
                )
            });
            group
        })
        .collect())
}

/// THE RESULTANT IS BUILT FROM OUR OWN NORMALIZED REACTIONS, AND NEVER FROM
/// `raw.equilibrium`. That is a decision, not an accident of how this was
/// written.
///
/// The kernel ships an `equilibrium` summary with an `appliedForceSum`, a
/// `reactionForceSum`, a `maxImbalance` and an `equilibriumOk` flag, and on any
/// frame with an ELASTIC support that summary is wrong: its reaction sum OMITS
/// the spring reactions, so it reports a perfectly balanced solve as out of
/// equilibrium, with `maxImbalance` equal to exactly the force the springs are
/// carrying. Measured, and pinned in `qualification/elastic-support.test.ts`.
///
/// The `reactions` ARRAY is complete — spring reactions are in it, once, with
/// the ordinary sign convention — so summing that array is right where trusting
/// the summary would be wrong. Nothing downstream may substitute the kernel's
/// own verdict for this one.
fn reaction_resultant(
    compiled: &CompiledStabileoModel,
    reactions: &[KernelReaction],
) -> PalletResult<Resultant> {
    let support_by_id: HashMap<&str, &crate::schema::FrameSupport> = compiled
        .frame
        .supports
        .iter()
        .map(|support| (support.support_id.as_str(), support))
        .collect();
    let node_by_id: HashMap<&str, Vec3> = compiled
        .frame
        .nodes
        .iter()
        .map(|node| (node.node_id.as_str(), node.position.vec()))
        .collect();
    let mut force = Vec3::ZERO;
    let mut moment = Vec3::ZERO;
    for reaction in reactions {
        let Some(support) = support_by_id.get(reaction.support_id.as_str()) else {
            return corrupt(format!("unknown support {}", reaction.support_id));
        };
        let Some(point) = node_by_id.get(support.node_id.as_str()) else {
            return corrupt(format!("unknown node {}", support.node_id));
        };
        force = add(force, reaction.force.vec());
        moment = add(moment, add(reaction.moment.vec(), cross(*point, reaction.force.vec())));
    }
    let newton = |value: f64| -> PalletResult<Quantity> {
        Ok(Quantity { unit: "N".to_string(), value: finite(value, "reaction resultant")? })
    };
    let newton_metre = |value: f64| -> PalletResult<Quantity> {
        Ok(Quantity { unit: "N_m".to_string(), value: finite(value, "reaction resultant")? })
    };
    Ok(Resultant {
        force: ResultantAxes { x: newton(force.x)?, y: newton(force.y)?, z: newton(force.z)? },
        moment: ResultantAxes {
            x: newton_metre(moment.x)?,
            y: newton_metre(moment.y)?,
            z: newton_metre(moment.z)?,
        },
    })
}

pub fn normalize_static_result(
    compiled: &CompiledStabileoModel,
    raw: &AnalysisResults3D,
) -> PalletResult<KernelResult> {
    let node_responses = normalize_nodes(compiled, raw)?;
    let reactions = normalize_reactions(compiled, raw)?;
    let element_end_forces = normalize_element_forces(compiled, raw)?;
    let reaction_result = reaction_resultant(compiled, &reactions)?;
    // THE FRAME'S OWN RADIUS about the origin its moments are taken about: the
    // longest lever any force residual can be acting on, and therefore the length
    // that turns a force tolerance into a moment tolerance. Derived from the
    // model rather than assumed, because a millinewton-metre means one thing on a
    // pallet and another on a bridge.
    let characteristic_length_m = compiled
        .frame
        .nodes
        .iter()
        .fold(0.0_f64, |longest, node| longest.max(node.position.vec().hypot3()));
    let audit = audit_equilibrium(
        &compiled.applied_resultant,
        &reaction_result,
        characteristic_length_m,
    )?;
    if !audit.accepted {
        // BY HOW MUCH, AND AGAINST WHAT. A residual a hair over the bound is a
        // conditioning question; one several orders over is a model that did not
        // balance, and the two want opposite responses. Reporting neither made this
        // the most expensive sentence in the pipeline to act on.
        let applied = &audit.applied;
        let reactions = &audit.reactions;
        let axes_norm = |axes: &ResultantAxes| -> f64 {
            Vec3 { x: axes.x.value, y: axes.y.value, z: axes.z.value }.hypot3()
        };
        // The kernel's RAW diagnostics, not the normalized ones. `over_constrained_dof`
        // has no entry in the diagnostic map, so `normalize_structured_diagnostics`
        // drops it — the kernel says the model is constrained more times than it
        // needs and nothing downstream ever hears it. Here, at least, the sentence
        // that refuses the answer repeats what the solver said, WITH the nodes it
        // named: a redundancy has a location, and the location is the fix.
        let mut by_code: Vec<(String, Vec<String>)> = Vec::new();
        for entry in &raw.structured_diagnostics {
            let code = format!(
                "{}:{}",
                severity_name(entry.severity),
                diagnostic_code_name(entry.code)
            );
            let where_node = match entry.node_ids.first() {
                Some(node_id) => Some(compiled.ids.nodes.stable(*node_id)?.to_string()),
                None => None,
            };
            match by_code.iter_mut().find(|(existing, _)| *existing == code) {
                Some((_, seen)) => {
                    if let Some(where_node) = where_node {
                        seen.push(where_node);
                    }
                }
                None => by_code.push((code, where_node.into_iter().collect())),
            }
        }
        let kernel_says = by_code
            .iter()
            .map(|(code, where_nodes)| {
                if where_nodes.is_empty() {
                    code.clone()
                } else {
                    let mut unique: Vec<&String> = Vec::new();
                    for node in where_nodes {
                        if !unique.contains(&node) {
                            unique.push(node);
                        }
                    }
                    let listed: Vec<&str> =
                        unique.iter().take(6).map(|node| node.as_str()).collect();
                    format!("{code}x{} at {}", where_nodes.len(), listed.join(", "))
                }
            })
            .collect::<Vec<String>>()
            .join("; ");
        return corrupt(format!(
            "equilibrium residual exceeds the frozen numerical profile \
             (force {} N against {} N \
             on {} N applied / {} N reacted; \
             moment {} N·m against {} N·m \
             on {} / {} N·m \
             (residual by axis {}, {}, {} N·m — the two resultants \
             SUM to zero at equilibrium, they do not difference) \
             over a {} m frame; {}; kernel says [{}])",
            to_precision(audit.force_residual_norm, 4),
            to_precision(audit.force_tolerance, 4),
            to_precision(axes_norm(&applied.force), 6),
            to_precision(axes_norm(&reactions.force), 6),
            to_precision(audit.moment_residual_norm, 4),
            to_precision(audit.moment_tolerance, 4),
            to_precision(axes_norm(&applied.moment), 6),
            to_precision(axes_norm(&reactions.moment), 6),
            // WHICH AXIS the moment went missing about, because the norm alone points
            // nowhere: a residual about the VERTICAL is a plan-twist the frame never
            // resisted, and one about a horizontal axis is an overturning couple the
            // supports failed to redistribute. Those are different faults with
            // different fixes, and a single number cannot tell them apart.
            to_precision(applied.moment.x.value + reactions.moment.x.value, 4),
            to_precision(applied.moment.y.value + reactions.moment.y.value, 4),
            to_precision(applied.moment.z.value + reactions.moment.z.value, 4),
            to_precision(characteristic_length_m, 4),
            if reactions.force.y.value == 0.0 {
                "no reactions".to_string()
            } else {
                format!("{} supports", compiled.frame.supports.len())
            },
            if kernel_says.is_empty() { "nothing" } else { kernel_says.as_str() },
        ));
    }
    let node_ids = &compiled.ids.nodes;
    let element_ids = &compiled.ids.elements;
    let diagnostics = normalize_structured_diagnostics(
        &raw.structured_diagnostics,
        &|id| node_ids.stable(id).map(str::to_string),
        &|id| element_ids.stable(id).map(str::to_string),
    )?;
    Ok(KernelResult {
        schema_version: "FP_KERNEL_RESULT_1".to_string(),
        request_id: compiled.request_id.clone(),
        active_state_id: compiled.active_state_id.clone(),
        active_state_hash: compiled.active_state_hash.clone(),
        node_responses,
        reactions,
        element_end_forces,
        connector_responses: connector_responses(compiled, raw)?
            .into_iter()
            .map(|response| serde_json::to_value(response).unwrap_or(Value::Null))
            .collect(),
        constraint_forces: normalize_constraint_forces(compiled, &raw.constraint_forces)?
            .into_iter()
            .map(|group| serde_json::to_value(group).unwrap_or(Value::Null))
            .collect(),
        applied_resultant: compiled.applied_resultant.clone(),
        reaction_resultant: reaction_result,
        force_residual: audit.force_residual,
        moment_residual: audit.moment_residual,
        diagnostics,
        // PORTING.md rule 7: the TS stamps `sha256CanonicalSync(payload)` here
        // for the application's own identity. The loop computes no hashes.
        result_hash: "internal".to_string(),
    })
}
