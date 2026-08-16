//! `projectPackageContacts` — package contact patches onto the pallet deck.
//!
//! Literal port of `packages/analysis/pallet/src/contact-projection.ts`. The
//! statement order, the accumulation order of every fold, and the error
//! sentences are the reference's; see `PORTING.md`.
//!
//! Hashing (PORTING.md rule 7): the reference stamps `sourceFrameGeometryHash`
//! and `projectionSha256` with sha256 envelopes. Inside this crate the
//! projection never leaves the process, so those two carry the literal
//! placeholder `"internal"`; `sourceFrameHash` and `memberMapHash` are echoed
//! from their inputs. The reference's `Schema.parse()` calls are validation
//! only and are skipped.

use std::collections::{HashMap, HashSet};

use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

use crate::schema::{
    AnalysisFrame, ContactFaceSystem, ContactMapEntry, ContactResponsePoint, Extra, FrameConnector,
    FrameElement, FrameLoad, FrameLoadApplication, FrameNode, FrameSupport,
    NumericalAcceptanceProfile, PalletContactProjectionResult, PalletTopContactPatch, Quantity,
    Resultant, ResultantAxes, ResultantConservationAudit, Tagged3,
};
use crate::types::{PalletError, PalletResult, Vec3};

use super::compare_canonical_utf8;

// ---------------------------------------------------------------------------
// Member map
// ---------------------------------------------------------------------------

/// The pallet member map, as much of it as this projection READS.
///
/// PORT-QUESTION: `PalletMemberMap` has no counterpart in `schema.rs`, so the
/// transport shape is modelled here rather than added to the shared module.
/// `projectPackageContacts` touches exactly four things on it — `entries[].kind`
/// (to select `TOP_DECKBOARD` members), `entries[].memberId`,
/// `entries[].segmentIds` (element → member lookup), `entries[].runAxis` (which
/// plan axis the board runs along), and `mapSha256` (echoed into the result as
/// `memberMapHash`). Everything else the application's schema carries
/// (`materialStateSha256`, `localRange`, `schemaVersion`) round-trips through
/// the flattened `extra` maps untouched.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PalletMemberMapEntry {
    pub member_id: String,
    pub kind: String,
    pub run_axis: String,
    pub segment_ids: Vec<String>,
    #[serde(flatten)]
    pub extra: Extra,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PalletMemberMap {
    pub entries: Vec<PalletMemberMapEntry>,
    /// Application identity; carried, never recomputed inside the crate.
    #[serde(rename = "mapSha256")]
    pub map_sha256: String,
    #[serde(flatten)]
    pub extra: Extra,
}

// ---------------------------------------------------------------------------
// Top surfaces
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
struct SurfaceSegment {
    element_id: String,
    member_id: String,
    start_node_id: String,
    end_node_id: String,
    start: Vec3,
    end: Vec3,
    min_x: f64,
    max_x: f64,
    min_z: f64,
    max_z: f64,
    top_y: f64,
}

/// `{ dimensionY, dimensionZ }` — the rectangular section the element's own
/// stiffness properties imply: `dimensionZ = sqrt(12·I_yy / A)` is the width
/// that reproduces the second moment about y, and `dimensionY = A / dimensionZ`
/// the depth that then reproduces the area.
fn dimensions(element: &FrameElement) -> (f64, f64) {
    let dimension_z = (12.0 * element.second_moment_yy_m4 / element.area_m2).sqrt();
    let dimension_y = element.area_m2 / dimension_z;
    (dimension_y, dimension_z)
}

fn top_surfaces(
    frame: &AnalysisFrame,
    member_map: &PalletMemberMap,
) -> PalletResult<Vec<SurfaceSegment>> {
    // `new Map(...)` keeps the LAST entry for a duplicate key; so does
    // `HashMap::extend`/`collect`.
    let node_by_id: HashMap<&str, &FrameNode> =
        frame.nodes.iter().map(|node| (node.node_id.as_str(), node)).collect();
    let top_members: HashSet<&str> = member_map
        .entries
        .iter()
        .filter(|entry| entry.kind == "TOP_DECKBOARD")
        .map(|entry| entry.member_id.as_str())
        .collect();
    let mut surfaces: Vec<SurfaceSegment> = Vec::new();
    for element in &frame.elements {
        let member_id = member_map
            .entries
            .iter()
            .find(|entry| entry.segment_ids.iter().any(|id| id == &element.element_id))
            .map(|entry| entry.member_id.as_str());
        let Some(member_id) = member_id else { continue };
        if !top_members.contains(member_id) {
            continue;
        }
        let start = node_by_id.get(element.start_node_id.as_str()).map(|node| node.position.vec());
        let end = node_by_id.get(element.end_node_id.as_str()).map(|node| node.position.vec());
        let (Some(start), Some(end)) = (start, end) else {
            return Err(PalletError::sentence(format!(
                "CONTACT_ELEMENT_NODE_MISSING:{}",
                element.element_id
            )));
        };
        let (dimension_y, dimension_z) = dimensions(element);
        let run_axis = member_map
            .entries
            .iter()
            .find(|entry| entry.member_id == member_id)
            .map(|entry| entry.run_axis.as_str());
        let half_width = dimension_z / 2.0;
        surfaces.push(SurfaceSegment {
            element_id: element.element_id.clone(),
            member_id: member_id.to_string(),
            start_node_id: element.start_node_id.clone(),
            end_node_id: element.end_node_id.clone(),
            start,
            end,
            min_x: if run_axis == Some("X") { start.x.min(end.x) } else { start.x - half_width },
            max_x: if run_axis == Some("X") { start.x.max(end.x) } else { start.x + half_width },
            min_z: if run_axis == Some("Z") { start.z.min(end.z) } else { start.z - half_width },
            max_z: if run_axis == Some("Z") { start.z.max(end.z) } else { start.z + half_width },
            top_y: start.y.max(end.y) + dimension_y / 2.0,
        });
    }
    surfaces.sort_by(|left, right| compare_canonical_utf8(&left.element_id, &right.element_id));
    Ok(surfaces)
}

fn point_inside(surface: &SurfaceSegment, x: f64, z: f64, tolerance: f64) -> bool {
    x >= surface.min_x - tolerance
        && x <= surface.max_x + tolerance
        && z >= surface.min_z - tolerance
        && z <= surface.max_z + tolerance
}

// ---------------------------------------------------------------------------
// Plan-space clipping
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy)]
struct PlanPoint {
    x: f64,
    z: f64,
}

fn oriented_rectangle(center_x: f64, center_z: f64, half_x: f64, half_z: f64, angle: f64) -> Vec<PlanPoint> {
    let cosine = angle.cos();
    let sine = angle.sin();
    [(-half_x, -half_z), (half_x, -half_z), (half_x, half_z), (-half_x, half_z)]
        .into_iter()
        .map(|(local_x, local_z)| PlanPoint {
            x: center_x + local_x * cosine - local_z * sine,
            z: center_z + local_x * sine + local_z * cosine,
        })
        .collect()
}

fn clip_boundary(
    polygon: &[PlanPoint],
    inside: impl Fn(&PlanPoint) -> bool,
    intersect: impl Fn(&PlanPoint, &PlanPoint) -> PlanPoint,
) -> Vec<PlanPoint> {
    if polygon.is_empty() {
        return Vec::new();
    }
    let mut output: Vec<PlanPoint> = Vec::new();
    for index in 0..polygon.len() {
        let start = &polygon[index];
        let end = &polygon[(index + 1) % polygon.len()];
        let start_inside = inside(start);
        let end_inside = inside(end);
        if start_inside && end_inside {
            output.push(*end);
        } else if start_inside {
            output.push(intersect(start, end));
        } else if end_inside {
            output.push(intersect(start, end));
            output.push(*end);
        }
    }
    output
}

fn at_x(x: f64, start: &PlanPoint, end: &PlanPoint) -> PlanPoint {
    let ratio = (x - start.x) / (end.x - start.x);
    PlanPoint { x, z: start.z + (end.z - start.z) * ratio }
}

fn at_z(z: f64, start: &PlanPoint, end: &PlanPoint) -> PlanPoint {
    let ratio = (z - start.z) / (end.z - start.z);
    PlanPoint { x: start.x + (end.x - start.x) * ratio, z }
}

fn clip_to_surface(polygon: &[PlanPoint], surface: &SurfaceSegment) -> Vec<PlanPoint> {
    let mut clipped: Vec<PlanPoint> = polygon.to_vec();
    clipped = clip_boundary(&clipped, |point| point.x >= surface.min_x, |start, end| at_x(surface.min_x, start, end));
    clipped = clip_boundary(&clipped, |point| point.x <= surface.max_x, |start, end| at_x(surface.max_x, start, end));
    clipped = clip_boundary(&clipped, |point| point.z >= surface.min_z, |start, end| at_z(surface.min_z, start, end));
    clip_boundary(&clipped, |point| point.z <= surface.max_z, |start, end| at_z(surface.max_z, start, end))
}

struct AreaCentroid {
    area: f64,
    x: f64,
    z: f64,
}

fn polygon_area_centroid(polygon: &[PlanPoint]) -> Option<AreaCentroid> {
    if polygon.len() < 3 {
        return None;
    }
    let mut double_area = 0.0f64;
    let mut x_numerator = 0.0f64;
    let mut z_numerator = 0.0f64;
    for index in 0..polygon.len() {
        let left = &polygon[index];
        let right = &polygon[(index + 1) % polygon.len()];
        let cross_value = left.x * right.z - right.x * left.z;
        double_area += cross_value;
        x_numerator += (left.x + right.x) * cross_value;
        z_numerator += (left.z + right.z) * cross_value;
    }
    // `Number.EPSILON` is the f64 machine epsilon.
    if double_area.abs() <= f64::EPSILON {
        return None;
    }
    Some(AreaCentroid {
        area: double_area.abs() / 2.0,
        x: x_numerator / (3.0 * double_area),
        z: z_numerator / (3.0 * double_area),
    })
}

// ---------------------------------------------------------------------------
// Contact weighting
// ---------------------------------------------------------------------------

/// One bearing sample: a plan position, a raw weight, and the surface it bears
/// on. The reference carries the surface by reference; the port carries its
/// index into the (already canonically sorted) surface list.
#[derive(Debug, Clone, Copy)]
struct Intersection {
    surface_index: usize,
    raw_weight: f64,
    x: f64,
    z: f64,
}

/// The nearest bearing point to a plan position, over every top surface.
///
/// Surfaces arrive sorted by element ID, so an exact-distance tie resolves to
/// the first in canonical order — deterministic, and therefore hash-stable.
fn nearest_surface_point(
    surfaces: &[SurfaceSegment],
    x: f64,
    z: f64,
) -> PalletResult<(usize, f64, f64)> {
    let mut best: Option<(usize, f64, f64, f64)> = None;
    for (index, surface) in surfaces.iter().enumerate() {
        let clamped_x = surface.min_x.max(surface.max_x.min(x));
        let clamped_z = surface.min_z.max(surface.max_z.min(z));
        let distance = (clamped_x - x).hypot(clamped_z - z);
        if best.is_none() || distance < best.unwrap().3 {
            best = Some((index, clamped_x, clamped_z, distance));
        }
    }
    let Some(best) = best else {
        return Err(PalletError::sentence("CONTACT_PROJECTION_NO_TOP_SURFACES"));
    };
    Ok((best.0, best.1, best.2))
}

/// Point samples keep their weight even where the deck is not.
///
/// A sample that lands in the gap between boards or past the deck edge used to
/// be DROPPED, and the drop concentrated: a four-corner bag with three corners
/// overhanging put its ENTIRE force on the one surviving corner, and the
/// moment correction then reproduced the missing lever as a concentrated
/// torque about that one board's own roll axis. Measured on a live project:
/// 1,383.9 N at a single point plus +372 N·m on one deckboard, against
/// ~1.6 kN·m/rad of total roll restraint — the board answered with 1.18 rad
/// and the drift guard refused the solve. The load the frame felt was not the
/// load the input described.
///
/// The physics of the clamp: these samples describe COMPLIANT contact (bags,
/// chime rings), and a compliant footprint's overhanging corner droops until
/// it bears on the nearest deck that exists. So a missing sample moves to the
/// nearest point of the nearest surface, keeping its weight; the moment
/// correction downstream restores the contact's true resultant exactly, now
/// through levers the size of the overhang instead of the patch. A sample
/// that bears where it stands is untouched, so a fully-bearing contact
/// projects byte-identically to before.
///
/// A contact with NO bearing sample stays empty — the caller's
/// CONTACT_HAS_NO_BEARING_SURFACE blocker distinguishes a package off the
/// pallet from a corner over its edge, and clamping the former would mask it.
fn clamped_point_samples(
    samples: &[(f64, f64, f64)],
    surfaces: &[SurfaceSegment],
    tolerance: f64,
) -> PalletResult<Vec<Intersection>> {
    let assigned: Vec<(f64, f64, f64, Option<usize>)> = samples
        .iter()
        .map(|&(x, z, raw_weight)| {
            (
                x,
                z,
                raw_weight,
                surfaces.iter().position(|candidate| point_inside(candidate, x, z, tolerance)),
            )
        })
        .collect();
    if !assigned.iter().any(|entry| entry.3.is_some()) {
        return Ok(Vec::new());
    }
    let mut output: Vec<Intersection> = Vec::with_capacity(assigned.len());
    for (x, z, raw_weight, surface) in assigned {
        if let Some(surface_index) = surface {
            output.push(Intersection { surface_index, raw_weight, x, z });
            continue;
        }
        let (surface_index, nearest_x, nearest_z) = nearest_surface_point(surfaces, x, z)?;
        output.push(Intersection { surface_index, raw_weight, x: nearest_x, z: nearest_z });
    }
    Ok(output)
}

/// The shared `PalletTopContactPatch` carries every variant's dimensions as
/// `Option` because one struct backs all five kinds; the application's zod
/// discriminated union makes each one REQUIRED for its own kind. A missing
/// field is a malformed patch, not a branch the reference has — so it fails
/// loudly here rather than silently substituting a default.
fn required_quantity(
    value: Option<&Quantity>,
    contact_id: &str,
    field: &str,
) -> PalletResult<f64> {
    match value {
        Some(quantity) => Ok(quantity.value),
        None => Err(PalletError::new(
            "CONTACT_PATCH_FIELD_MISSING",
            format!("CONTACT_PATCH_FIELD_MISSING:{contact_id}:{field}"),
        )),
    }
}

fn contact_weights(
    contact: &PalletTopContactPatch,
    surfaces: &[SurfaceSegment],
    tolerance: f64,
) -> PalletResult<Vec<Intersection>> {
    if contact.kind == "POINT" {
        return Ok(surfaces
            .iter()
            .enumerate()
            .filter(|(_, surface)| point_inside(surface, contact.center.x, contact.center.z, tolerance))
            .map(|(surface_index, _)| Intersection {
                surface_index,
                raw_weight: 1.0,
                x: contact.center.x,
                z: contact.center.z,
            })
            .collect());
    }
    if contact.kind == "RECTANGULAR_PATCH" || contact.kind == "ORIENTATION_BAND" {
        let half_x = if contact.kind == "RECTANGULAR_PATCH" {
            required_quantity(contact.half_size_x.as_ref(), &contact.contact_id, "halfSizeX")?
        } else {
            required_quantity(contact.half_length.as_ref(), &contact.contact_id, "halfLength")?
        };
        let half_z = if contact.kind == "RECTANGULAR_PATCH" {
            required_quantity(contact.half_size_z.as_ref(), &contact.contact_id, "halfSizeZ")?
        } else {
            required_quantity(contact.half_width.as_ref(), &contact.contact_id, "halfWidth")?
        };
        let orientation =
            required_quantity(contact.orientation.as_ref(), &contact.contact_id, "orientation")?;
        let polygon =
            oriented_rectangle(contact.center.x, contact.center.z, half_x, half_z, orientation);
        let mut output: Vec<Intersection> = Vec::new();
        for (surface_index, surface) in surfaces.iter().enumerate() {
            let Some(projection) = polygon_area_centroid(&clip_to_surface(&polygon, surface)) else {
                continue;
            };
            if projection.area <= tolerance.powi(2) {
                continue;
            }
            output.push(Intersection {
                surface_index,
                raw_weight: projection.area,
                x: projection.x,
                z: projection.z,
            });
        }
        return Ok(output);
    }
    if contact.kind == "PRESSURE_FIELD" {
        // WHAT THE FIELD'S OWN DATA SAYS decides the projection. The field is a
        // boundary polygon with one pressure weight per boundary point. When
        // every weight is the same — the only field the estimator's bag geometry
        // produces (`shape-geometry.ts`, `1 / corners.length` each) — the data
        // states a UNIFORM pressure over the boundary, and a uniform pressure's
        // exact projection is the area clip: force per surface proportional to
        // covered area, applied at each clip's own centroid. That is the same
        // treatment a rigid box patch gets, and it is what keeps a bag spanning
        // three deckboards from loading two of them at their corners and the
        // third not at all.
        //
        // Vertex quadrature — the whole force as point loads at the corner
        // samples — was measured doing exactly that on a live project: a bag
        // whose corners overhung the deck put 1,383.9 N on ONE surviving corner
        // with +372 N·m of manufactured torque about that deckboard's roll axis,
        // and the deck answered with 1.18 rad. Point loads at the corners of a
        // patch are a quadrature OF the field, and a quadrature is only faithful
        // while its points bear.
        //
        // A NON-uniform field is known only at its sample points (the schema
        // defines no interpolation between them), so the authored quadrature
        // stands — with missing samples clamped, never dropped.
        let samples = contact.samples.as_deref().unwrap_or(&[]);
        let weights: Vec<f64> = samples.iter().map(|sample| sample.normalized_weight).collect();
        // `weights.every(w => w === weights[0])` on an empty array is `true` in
        // JS; the zod schema forbids an empty sample list, so this reads the
        // same either way.
        if weights.iter().all(|weight| Some(*weight) == weights.first().copied()) {
            let boundary = contact.boundary.as_deref().unwrap_or(&[]);
            let polygon: Vec<PlanPoint> =
                boundary.iter().map(|point| PlanPoint { x: point.x, z: point.z }).collect();
            let mut output: Vec<Intersection> = Vec::new();
            for (surface_index, surface) in surfaces.iter().enumerate() {
                let Some(projection) = polygon_area_centroid(&clip_to_surface(&polygon, surface))
                else {
                    continue;
                };
                if projection.area <= tolerance.powi(2) {
                    continue;
                }
                output.push(Intersection {
                    surface_index,
                    raw_weight: projection.area,
                    x: projection.x,
                    z: projection.z,
                });
            }
            return Ok(output);
        }
        let points: Vec<(f64, f64, f64)> = samples
            .iter()
            .map(|sample| (sample.point.x, sample.point.z, sample.normalized_weight))
            .collect();
        return clamped_point_samples(&points, surfaces, tolerance);
    }
    // ANNULAR_CHIME — the only remaining variant of the discriminated union.
    let inner = required_quantity(contact.inner_radius.as_ref(), &contact.contact_id, "innerRadius")?;
    let outer = required_quantity(contact.outer_radius.as_ref(), &contact.contact_id, "outerRadius")?;
    let radius = (inner + outer) / 2.0;
    let points: Vec<(f64, f64, f64)> = (0..32)
        .map(|index| {
            let theta = 2.0 * std::f64::consts::PI * (index as f64) / 32.0;
            (contact.center.x + radius * theta.cos(), contact.center.z + radius * theta.sin(), 1.0)
        })
        .collect();
    clamped_point_samples(&points, surfaces, tolerance)
}

fn normalized_position(surface: &SurfaceSegment, x: f64, z: f64) -> PalletResult<f64> {
    let dx = surface.end.x - surface.start.x;
    let dz = surface.end.z - surface.start.z;
    let denominator = dx * dx + dz * dz;
    if denominator <= 0.0 {
        return Err(PalletError::sentence(format!(
            "CONTACT_ELEMENT_ZERO_LENGTH:{}",
            surface.element_id
        )));
    }
    Ok(0.0f64.max(1.0f64.min(((x - surface.start.x) * dx + (z - surface.start.z) * dz) / denominator)))
}

/// `Number.prototype.toFixed(4)`.
///
/// Rust's `{:.4}` rounds halfway cases to even where JS rounds them away from
/// zero, and prints `-0.0000` for a negative zero where JS prints `0.0000`.
/// Only the second case can be hit by real coordinates (a tie at the fourth
/// decimal of a metre is not producible by the geometry compiler), so the sign
/// of zero is normalized and the rest is left to the formatter. This feeds an
/// error message, never a number the gate compares.
fn to_fixed_4(value: f64) -> String {
    if value == 0.0 {
        return format!("{:.4}", 0.0);
    }
    format!("{value:.4}")
}

fn resultant(force: Vec3, moment: Vec3) -> Resultant {
    Resultant {
        force: ResultantAxes {
            x: Quantity { unit: "N".into(), value: force.x },
            y: Quantity { unit: "N".into(), value: force.y },
            z: Quantity { unit: "N".into(), value: force.z },
        },
        moment: ResultantAxes {
            x: Quantity { unit: "N_m".into(), value: moment.x },
            y: Quantity { unit: "N_m".into(), value: moment.y },
            z: Quantity { unit: "N_m".into(), value: moment.z },
        },
    }
}

fn load_id_of(load: &FrameLoad) -> &str {
    load.load_id()
}

fn node_id_of(node: &FrameNode) -> &str {
    node.node_id.as_str()
}

fn json_field<'a>(value: &'a Value, key: &str) -> &'a str {
    value.get(key).and_then(Value::as_str).unwrap_or("")
}

/// One bearing sample after weighting: the reference's `projected` entry.
#[derive(Debug, Clone, Copy)]
struct Projected {
    surface_index: usize,
    weight: f64,
    point: Vec3,
    weighted_force: Vec3,
    natural: f64,
}

// ---------------------------------------------------------------------------
// projectPackageContacts
// ---------------------------------------------------------------------------

pub fn project_package_contacts(
    frame: &AnalysisFrame,
    member_map: &PalletMemberMap,
    contacts: &[PalletTopContactPatch],
    numerical_profile: &NumericalAcceptanceProfile,
) -> PalletResult<PalletContactProjectionResult> {
    let surfaces = top_surfaces(frame, member_map)?;
    let node_position_by_id: HashMap<&str, Vec3> =
        frame.nodes.iter().map(|node| (node.node_id.as_str(), node.position.vec())).collect();
    let mut loads: Vec<FrameLoad> = Vec::new();
    let mut face_nodes: Vec<FrameNode> = Vec::new();
    let mut face_constraints: Vec<Value> = Vec::new();
    let mut face_connectors: Vec<FrameConnector> = Vec::new();
    let mut face_supports: Vec<FrameSupport> = Vec::new();
    let mut face_loads: Vec<FrameLoad> = Vec::new();
    let mut contact_map: Vec<ContactMapEntry> = Vec::new();
    let mut input_force = Vec3::ZERO;
    let mut input_moment = Vec3::ZERO;
    let mut projected_force = Vec3::ZERO;
    let mut projected_moment = Vec3::ZERO;
    for contact in contacts {
        let intersections =
            contact_weights(contact, &surfaces, numerical_profile.geometry_tolerance_m)?;
        let total_raw_weight =
            intersections.iter().fold(0.0f64, |sum, entry| sum + entry.raw_weight);
        if total_raw_weight <= 0.0 {
            // WHERE it is and where the deck is, because "no bearing surface" on its
            // own cannot distinguish a package that overhangs the pallet from a load
            // compiled in the wrong coordinate frame — and the answers are a blocker
            // and a bug respectively.
            let extent = |values: &[f64]| -> String {
                if values.is_empty() {
                    return "none".to_string();
                }
                let minimum = values.iter().fold(f64::INFINITY, |left, right| left.min(*right));
                let maximum = values.iter().fold(f64::NEG_INFINITY, |left, right| left.max(*right));
                format!("{}…{}", to_fixed_4(minimum), to_fixed_4(maximum))
            };
            let x_extent: Vec<f64> =
                surfaces.iter().flat_map(|surface| [surface.min_x, surface.max_x]).collect();
            let z_extent: Vec<f64> =
                surfaces.iter().flat_map(|surface| [surface.min_z, surface.max_z]).collect();
            return Err(PalletError::sentence(format!(
                "CONTACT_HAS_NO_BEARING_SURFACE:{} (centre x={} z={}; {} top surfaces spanning x={} z={})",
                contact.contact_id,
                to_fixed_4(contact.center.x),
                to_fixed_4(contact.center.z),
                surfaces.len(),
                extent(&x_extent),
                extent(&z_extent),
            )));
        }
        // PORT-QUESTION: the shared `PalletTopContactPatch` has no `sourceId`
        // field, but the application's `ContactCommon` makes it required and the
        // projection stamps it into every emitted load's `application.sourceId`.
        // Read from the flattened `extra` map and fail loudly when absent.
        let Some(source_id) = contact.extra.get("sourceId").and_then(Value::as_str) else {
            return Err(PalletError::new(
                "CONTACT_PATCH_FIELD_MISSING",
                format!("CONTACT_PATCH_FIELD_MISSING:{}:sourceId", contact.contact_id),
            ));
        };
        let source_id = source_id.to_string();
        let force = Vec3 { x: contact.force.x, y: contact.force.y, z: contact.force.z };
        let desired_lever_moment = contact.center.vec().cross(force);
        input_force.x += force.x;
        input_force.y += force.y;
        input_force.z += force.z;
        input_moment.x += desired_lever_moment.x + contact.free_moment.x;
        input_moment.y += desired_lever_moment.y + contact.free_moment.y;
        input_moment.z += desired_lever_moment.z + contact.free_moment.z;
        // THE LOAD SPLITS BY DIRECTION, because the two halves travel differently.
        //
        // The VERTICAL force is pressure, and pressure belongs to the rigid face
        // system built below: applied to a face node standing on the patch's own
        // bearing stiffness, so the deck's share of it redistributes by the deck's
        // actual compliance inside the solve. The HORIZONTAL forces are friction
        // drag on the deck surface and keep the direct nodal projection, with the
        // moment corrections recomputed against the horizontal system's own
        // levers (a surface-height shear force still makes a roll couple about a
        // board's axis — that part is real statics, not an artifact).
        let horizontal_force = Vec3 { x: force.x, y: 0.0, z: force.z };
        let desired_horizontal_lever = contact.center.vec().cross(horizontal_force);
        let desired_horizontal_moment = Vec3 {
            x: desired_horizontal_lever.x,
            y: desired_horizontal_lever.y + contact.free_moment.y,
            z: desired_horizontal_lever.z,
        };
        let mut weighted_lever = Vec3::ZERO;
        let mut projected: Vec<Projected> = Vec::with_capacity(intersections.len());
        for intersection in &intersections {
            let weight = intersection.raw_weight / total_raw_weight;
            let surface = &surfaces[intersection.surface_index];
            let point = Vec3 { x: intersection.x, y: surface.top_y, z: intersection.z };
            let weighted_force = Vec3 {
                x: horizontal_force.x * weight,
                y: 0.0,
                z: horizontal_force.z * weight,
            };
            let lever = point.cross(weighted_force);
            weighted_lever.x += lever.x;
            weighted_lever.y += lever.y;
            weighted_lever.z += lever.z;
            let natural = normalized_position(surface, intersection.x, intersection.z)?;
            projected.push(Projected {
                surface_index: intersection.surface_index,
                weight,
                point,
                weighted_force,
                natural,
            });
        }
        let moment_correction = Vec3 {
            x: desired_horizontal_moment.x - weighted_lever.x,
            y: desired_horizontal_moment.y - weighted_lever.y,
            z: desired_horizontal_moment.z - weighted_lever.z,
        };
        let mut response_points: Vec<ContactResponsePoint> = Vec::with_capacity(projected.len());
        for (index, entry) in projected.iter().enumerate() {
            let surface = &surfaces[entry.surface_index];
            let natural = entry.natural;
            let axis_point = Vec3 {
                x: surface.start.x + (surface.end.x - surface.start.x) * natural,
                y: surface.start.y + (surface.end.y - surface.start.y) * natural,
                z: surface.start.z + (surface.end.z - surface.start.z) * natural,
            };
            let application = FrameLoadApplication {
                source_kind: "PACKAGE_CONTACT".into(),
                source_id: source_id.clone(),
                parent_member_id: Some(surface.member_id.clone()),
                contact_id: Some(contact.contact_id.clone()),
            };
            let start_force = Vec3 {
                x: entry.weighted_force.x * (1.0 - natural),
                y: entry.weighted_force.y * (1.0 - natural),
                z: entry.weighted_force.z * (1.0 - natural),
            };
            let end_force = Vec3 {
                x: entry.weighted_force.x * natural,
                y: entry.weighted_force.y * natural,
                z: entry.weighted_force.z * natural,
            };
            let endpoint_lever_start = surface.start.cross(start_force);
            let endpoint_lever_end = surface.end.cross(end_force);
            let desired_point_lever = entry.point.cross(entry.weighted_force);
            let nodal_moment = Vec3 {
                x: desired_point_lever.x + moment_correction.x * entry.weight
                    - endpoint_lever_start.x
                    - endpoint_lever_end.x,
                y: desired_point_lever.y + moment_correction.y * entry.weight
                    - endpoint_lever_start.y
                    - endpoint_lever_end.y,
                z: desired_point_lever.z + moment_correction.z * entry.weight
                    - endpoint_lever_start.z
                    - endpoint_lever_end.z,
            };
            let ordinal = format!("{index:04}");
            loads.push(FrameLoad::NodalForce {
                load_id: format!("load:contact:{}:{}:force-start", contact.contact_id, ordinal),
                node_id: surface.start_node_id.clone(),
                force: Tagged3::polar("N", start_force),
                application: Some(application.clone()),
            });
            loads.push(FrameLoad::NodalForce {
                load_id: format!("load:contact:{}:{}:force-end", contact.contact_id, ordinal),
                node_id: surface.end_node_id.clone(),
                force: Tagged3::polar("N", end_force),
                application: Some(application.clone()),
            });
            loads.push(FrameLoad::NodalMoment {
                load_id: format!("load:contact:{}:{}:moment", contact.contact_id, ordinal),
                node_id: surface.start_node_id.clone(),
                moment: Tagged3::axial("N_m", nodal_moment),
                application: Some(application.clone()),
            });
            projected_force.x += start_force.x + end_force.x;
            projected_force.y += start_force.y + end_force.y;
            projected_force.z += start_force.z + end_force.z;
            projected_moment.x += endpoint_lever_start.x + endpoint_lever_end.x + nodal_moment.x;
            projected_moment.y += endpoint_lever_start.y + endpoint_lever_end.y + nodal_moment.y;
            projected_moment.z += endpoint_lever_start.z + endpoint_lever_end.z + nodal_moment.z;
            response_points.push(ContactResponsePoint {
                response_point_id: format!("response:{}:{index:04}", contact.contact_id),
                element_id: surface.element_id.clone(),
                element_natural_coordinate: natural,
                global_point: Tagged3::point_m(entry.point),
                rigid_offset_from_element_axis: Tagged3::polar(
                    "m",
                    Vec3 {
                        x: entry.point.x - axis_point.x,
                        y: entry.point.y - axis_point.y,
                        z: entry.point.z - axis_point.z,
                    },
                ),
                normalized_contact_weight: entry.weight,
            });
        }
        contact_map
            .push(ContactMapEntry { contact_id: contact.contact_id.clone(), response_points });
        // THE RIGID FACE ON ITS OWN BEARING SPRINGS — the vertical half of the
        // contact, as structure rather than as prescribed nodal forces.
        //
        // The patch's bearing stiffness (`normalStiffnessNPerM`, the unit-load
        // compiler's own derived interface spring) is split over the deck's
        // element endpoint nodes by the same weights the load used to be, and a
        // face node — rigidly linked to a seat node above each bearing point —
        // stands on those springs carrying the patch's vertical force. What the
        // deck receives is then decided by the SOLVE: a board that rolls or sinks
        // away from the rigid face sheds its share to the patch's other bearings,
        // which is the restoring mechanism a real package provides. Measured
        // without it at 0.4 g: a leeward corner board rolled 0.113 rad against the
        // model's own 0.1 rad linear-validity bound, and no downstream round can
        // heal it because the partition advance consumes the response
        // geometrically.
        //
        // The face's in-plan freedoms (TX, TZ, RY) see no stiffness from
        // vertical-only springs and carry no load — the horizontal forces stay on
        // the deck — so they are grounded to keep the model non-singular; TY, RX
        // and RZ ride the spring array. A rotation axis the bearing points give no
        // plan-spread about (a single bearing node, or all bearings on one line)
        // is grounded too, by the profile's own geometric resolution — the same
        // reasoning as the settlement solver's degenerate-set fallback. A contact
        // carrying no vertical force builds no face at all: an OPEN interface is a
        // package that is not pressing, and bridging boards through it would
        // stiffen the deck with a package that is not there.
        if force.y < 0.0 {
            let separation_m = numerical_profile.geometry_tolerance_m;
            let face_node_id = format!("node:face:{}", contact.contact_id);
            let face_position = Vec3 {
                x: contact.center.x,
                y: projected
                    .iter()
                    .fold(f64::NEG_INFINITY, |top, entry| top.max(surfaces[entry.surface_index].top_y))
                    + 2.0 * separation_m,
                z: contact.center.z,
            };
            face_nodes.push(FrameNode {
                node_id: face_node_id.clone(),
                position: Tagged3::point_m(face_position),
            });
            let mut bearing_positions: Vec<(f64, f64)> = Vec::new();
            // ONE SEAT PER BEARING SAMPLE, AT THE SAMPLE'S TRUE POSITION — off the
            // board's axis, on its surface — because the arm is the mechanism.
            //
            // The springs' anchor rides the deck through a rigid link from the
            // segment's endpoint nodes, offset to the sample's own point, so the
            // spring force enters the board with its true lever held exactly
            // (ECCENTRIC_CONNECTION, the defect-15 primitive) AND the anchor MOVES
            // when the board ROLLS: a roll θ swings the off-axis arm d, the spring
            // answers with k·θ·d, and the face returns k·d² of roll-restoring
            // stiffness — the pressure-patch bedding, derived from the interface's
            // own spring and the patch's own geometry, no constants. The first cut
            // of this system anchored at the AXIS nodes, where d = 0: it restored
            // sink and was blind to roll, and the fff corner measured WORSE
            // (0.113 → 0.138 rad) because the axis anchoring also deleted the
            // vertical pressure's own off-axis torque. Both halves of a segment's
            // kinematics anchor the sample — start and end, split (1−t)/t — the
            // same statics the direct nodal split has always used.
            //
            // THE ARM CARRIES ONLY WHAT THE PHYSICS NEEDS: the sample's
            // PERPENDICULAR offset from the member's axis — across the board and up
            // to its surface, never along it. A vertical spring on that arm couples
            // to exactly one rotation: roll about the member's own axis (θ_axis ×
            // d_transverse = vertical stretch, so the face returns k·d² of derived
            // bedding stiffness against roll), while bending and torsion swing the
            // arm through motions the spring cannot see. The first off-axis cut
            // carried the AXIAL arm too, and a rigid axial arm turns the spring
            // into a parasitic bending restraint of k·(t·L)² — measured ~30× the
            // board's own EI/L — which corrupted gravity outright (186 N / 930 N·m
            // equilibrium leak, sparse_fallback_dense_lu). Each half of the
            // (1−t)/t split therefore anchors at its OWN endpoint's axial station,
            // wearing the same perpendicular arm.
            let mut sample_ordinal: usize = 0;
            for entry in &projected {
                let surface = &surfaces[entry.surface_index];
                let ordinal = format!("{sample_ordinal:04}");
                sample_ordinal += 1;
                let axis_point = Vec3 {
                    x: surface.start.x + (surface.end.x - surface.start.x) * entry.natural,
                    y: surface.start.y + (surface.end.y - surface.start.y) * entry.natural,
                    z: surface.start.z + (surface.end.z - surface.start.z) * entry.natural,
                };
                let perpendicular_arm = Vec3 {
                    x: entry.point.x - axis_point.x,
                    y: entry.point.y - axis_point.y,
                    z: entry.point.z - axis_point.z,
                };
                let halves: [(&str, &str, f64); 2] = [
                    (
                        "start",
                        surface.start_node_id.as_str(),
                        contact.normal_stiffness_n_per_m * entry.weight * (1.0 - entry.natural),
                    ),
                    (
                        "end",
                        surface.end_node_id.as_str(),
                        contact.normal_stiffness_n_per_m * entry.weight * entry.natural,
                    ),
                ];
                for (half, anchor_node_id, share) in halves {
                    if !(share > 0.0) {
                        continue;
                    }
                    let Some(anchor_position) = node_position_by_id.get(anchor_node_id).copied()
                    else {
                        return Err(PalletError::sentence(format!(
                            "CONTACT_FACE_BEARING_NODE_MISSING:{anchor_node_id}"
                        )));
                    };
                    let bearing_point = Vec3 {
                        x: anchor_position.x + perpendicular_arm.x,
                        y: anchor_position.y + perpendicular_arm.y,
                        z: anchor_position.z + perpendicular_arm.z,
                    };
                    bearing_positions.push((bearing_point.x, bearing_point.z));
                    let bearing_node_id = format!(
                        "node:face:{}:bearing:{}:{}",
                        contact.contact_id, ordinal, half
                    );
                    face_nodes.push(FrameNode {
                        node_id: bearing_node_id.clone(),
                        position: Tagged3::point_m(bearing_point),
                    });
                    face_constraints.push(json!({
                        "kind": "ECCENTRIC_CONNECTION",
                        "constraintId": format!("constraint:face:{}:bearing:{}:{}", contact.contact_id, ordinal, half),
                        "masterNodeId": anchor_node_id,
                        "slaveNodeId": bearing_node_id,
                        "polarOffset": {
                            "kind": "POLAR_VECTOR",
                            "unit": "m",
                            "x": perpendicular_arm.x,
                            "y": perpendicular_arm.y,
                            "z": perpendicular_arm.z,
                        },
                        "releases": { "tx": false, "ty": false, "tz": false, "rx": false, "ry": false, "rz": false },
                    }));
                    let seat_node_id =
                        format!("node:face:{}:seat:{}:{}", contact.contact_id, ordinal, half);
                    // Separated from its bearing along the spring's own axis so the
                    // kernel can see the axis — the interface-pair lesson, verbatim:
                    // the direction (0, 1, 0) is exact in floating point at any ε.
                    let seat_position = Vec3 {
                        x: bearing_point.x,
                        y: bearing_point.y + separation_m,
                        z: bearing_point.z,
                    };
                    face_nodes.push(FrameNode {
                        node_id: seat_node_id.clone(),
                        position: Tagged3::point_m(seat_position),
                    });
                    face_constraints.push(json!({
                        "kind": "ECCENTRIC_CONNECTION",
                        "constraintId": format!("constraint:face:{}:seat:{}:{}", contact.contact_id, ordinal, half),
                        "masterNodeId": face_node_id,
                        "slaveNodeId": seat_node_id,
                        "polarOffset": {
                            "kind": "POLAR_VECTOR",
                            "unit": "m",
                            "x": seat_position.x - face_position.x,
                            "y": seat_position.y - face_position.y,
                            "z": seat_position.z - face_position.z,
                        },
                        "releases": { "tx": false, "ty": false, "tz": false, "rx": false, "ry": false, "rz": false },
                    }));
                    face_connectors.push(FrameConnector {
                        connector_id: format!(
                            "connector:face:{}:seat:{}:{}",
                            contact.contact_id, ordinal, half
                        ),
                        node_i: seat_node_id,
                        node_j: bearing_node_id,
                        axial_stiffness: Some(Quantity {
                            unit: "N_per_m".into(),
                            value: share,
                        }),
                        shear_y_stiffness: None,
                        shear_z_stiffness: None,
                        torsion_stiffness: None,
                        bend_y_stiffness: None,
                        bend_z_stiffness: None,
                    });
                }
            }
            if bearing_positions.is_empty() {
                return Err(PalletError::sentence(format!(
                    "CONTACT_FACE_HAS_NO_BEARING_NODES:{}",
                    contact.contact_id
                )));
            }
            let mean_x = bearing_positions.iter().fold(0.0f64, |sum, position| sum + position.0)
                / bearing_positions.len() as f64;
            let mean_z = bearing_positions.iter().fold(0.0f64, |sum, position| sum + position.1)
                / bearing_positions.len() as f64;
            let spread_x = bearing_positions
                .iter()
                .fold(f64::NEG_INFINITY, |widest, position| widest.max((position.0 - mean_x).abs()));
            let spread_z = bearing_positions
                .iter()
                .fold(f64::NEG_INFINITY, |widest, position| widest.max((position.1 - mean_z).abs()));
            face_supports.push(FrameSupport {
                support_id: format!("support:face:{}", contact.contact_id),
                node_id: face_node_id.clone(),
                active: true,
                // [tx, ty, tz, rx, ry, rz]: in-plan freedoms grounded always; a tilt
                // axis is grounded only when the bearing points give it no plan-spread
                // to be restrained by (rx needs spread in z, rz needs spread in x).
                fixed_dofs: [
                    true,
                    false,
                    true,
                    spread_z <= separation_m,
                    true,
                    spread_x <= separation_m,
                ],
                prescribed_translations: None,
                prescribed_rotations: None,
                elastic_stiffness: None,
            });
            face_loads.push(FrameLoad::NodalForce {
                load_id: format!("load:face:{}:force", contact.contact_id),
                node_id: face_node_id.clone(),
                force: Tagged3::polar("N", Vec3 { x: 0.0, y: force.y, z: 0.0 }),
                // No `application`: the attribution schema names a single parent
                // member, and the face load belongs to every board the patch bears on
                // at once — its provenance is the contactId in its own loadId.
                application: None,
            });
            face_loads.push(FrameLoad::NodalMoment {
                load_id: format!("load:face:{}:moment", contact.contact_id),
                node_id: face_node_id.clone(),
                moment: Tagged3::axial(
                    "N_m",
                    Vec3 { x: contact.free_moment.x, y: 0.0, z: contact.free_moment.z },
                ),
                application: None,
            });
            let face_lever = face_position.cross(Vec3 { x: 0.0, y: force.y, z: 0.0 });
            projected_force.y += force.y;
            projected_moment.x += face_lever.x + contact.free_moment.x;
            projected_moment.y += face_lever.y;
            projected_moment.z += face_lever.z + contact.free_moment.z;
        }
    }
    loads.sort_by(|left, right| compare_canonical_utf8(load_id_of(left), load_id_of(right)));
    contact_map
        .sort_by(|left, right| compare_canonical_utf8(&left.contact_id, &right.contact_id));
    let force_residual = Vec3 {
        x: projected_force.x - input_force.x,
        y: projected_force.y - input_force.y,
        z: projected_force.z - input_force.z,
    };
    let moment_residual = Vec3 {
        x: projected_moment.x - input_moment.x,
        y: projected_moment.y - input_moment.y,
        z: projected_moment.z - input_moment.z,
    };
    let force_residual_norm_n = force_residual.hypot3();
    let moment_residual_norm_nm = moment_residual.hypot3();
    let input_force_norm = input_force.hypot3();
    let resultant_location_residual_m = if input_force_norm > numerical_profile.force_tolerance_n {
        moment_residual_norm_nm / input_force_norm
    } else {
        0.0
    };
    let audit = ResultantConservationAudit {
        input_resultant: resultant(input_force, input_moment),
        projected_resultant: resultant(projected_force, projected_moment),
        force_residual_norm_n,
        moment_residual_norm_nm,
        resultant_location_residual_m,
        accepted: force_residual_norm_n <= numerical_profile.force_tolerance_n
            && moment_residual_norm_nm <= numerical_profile.moment_tolerance_nm
            && resultant_location_residual_m <= numerical_profile.length_tolerance_m,
    };
    if !audit.accepted {
        return Err(PalletError::sentence("CONTACT_RESULTANT_CONSERVATION_FAILED"));
    }
    face_nodes.sort_by(|left, right| compare_canonical_utf8(node_id_of(left), node_id_of(right)));
    face_constraints.sort_by(|left, right| {
        compare_canonical_utf8(json_field(left, "constraintId"), json_field(right, "constraintId"))
    });
    face_connectors
        .sort_by(|left, right| compare_canonical_utf8(&left.connector_id, &right.connector_id));
    face_supports
        .sort_by(|left, right| compare_canonical_utf8(&left.support_id, &right.support_id));
    face_loads.sort_by(|left, right| compare_canonical_utf8(load_id_of(left), load_id_of(right)));
    // PORT-QUESTION: `PalletContactProjectionResult` in `schema.rs` names only
    // `schemaVersion`, `loads`, `faceSystem`, `contactMap` and `audit`; the
    // application's result additionally carries `sourceFrameHash`,
    // `sourceFrameGeometryHash`, `memberMapHash` and `projectionSha256`. They
    // are written into the struct's flattened `extra` map so the transport shape
    // stays whole. Per PORTING.md rule 7 the two freshly-derived hashes carry
    // the literal placeholder "internal"; the two echoed ones carry their input.
    let mut extra = Extra::new();
    extra.insert("sourceFrameHash".into(), Value::String(frame.frame_hash.clone()));
    extra.insert("sourceFrameGeometryHash".into(), Value::String("internal".into()));
    extra.insert("memberMapHash".into(), Value::String(member_map.map_sha256.clone()));
    extra.insert("projectionSha256".into(), Value::String("internal".into()));
    Ok(PalletContactProjectionResult {
        schema_version: "FP_PALLET_CONTACT_PROJECTION_RESULT_3".into(),
        loads,
        face_system: ContactFaceSystem {
            nodes: face_nodes,
            constraints: face_constraints,
            connectors: face_connectors,
            supports: face_supports,
            loads: face_loads,
        },
        contact_map,
        audit,
        extra,
    })
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::schema::ContactPressureSample;

    fn profile() -> NumericalAcceptanceProfile {
        NumericalAcceptanceProfile {
            schema_version: "FP_NUMERICAL_ACCEPTANCE_PROFILE_1".into(),
            profile_id: "profile:test".into(),
            profile_sha256: "internal".into(),
            geometry_tolerance_m: 1e-6,
            force_tolerance_n: 1e-6,
            moment_tolerance_nm: 1e-6,
            length_tolerance_m: 1e-6,
            complementarity_tolerance_n: 1e-6,
            coupled_iteration_limit: 32,
            coupled_translation_tolerance_m: 1e-6,
            coupled_rotation_tolerance_rad: 1e-6,
            coupled_load_share_tolerance: 1e-6,
        }
    }

    fn node(node_id: &str, x: f64, y: f64, z: f64) -> FrameNode {
        FrameNode { node_id: node_id.into(), position: Tagged3::point_m(Vec3 { x, y, z }) }
    }

    /// A 0.1 m wide × 0.02 m deep deckboard: `dimensionZ = sqrt(12·I_yy/A)`
    /// recovers the 0.1 m width and `dimensionY = A/dimensionZ` the 0.02 m
    /// depth, so `halfWidth` is 0.05 m and `topY` is the node y plus 0.01 m.
    fn deckboard(element_id: &str, start_node_id: &str, end_node_id: &str) -> FrameElement {
        let width = 0.1f64;
        let depth = 0.02f64;
        FrameElement {
            element_id: element_id.into(),
            start_node_id: start_node_id.into(),
            end_node_id: end_node_id.into(),
            area_m2: width * depth,
            shear_area_y_m2: width * depth * 5.0 / 6.0,
            shear_area_z_m2: width * depth * 5.0 / 6.0,
            torsional_constant_m4: 1.0e-8,
            second_moment_yy_m4: depth * width * width * width / 12.0,
            second_moment_zz_m4: width * depth * depth * depth / 12.0,
            elastic_modulus: Quantity { unit: "Pa".into(), value: 9.0e9 },
            shear_modulus: Quantity { unit: "Pa".into(), value: 6.0e8 },
            local_y_axis: Tagged3::polar("dimensionless", Vec3 { x: 0.0, y: 1.0, z: 0.0 }),
            release_start: [false; 6],
            release_end: [false; 6],
        }
    }

    /// Two parallel top deckboards, each one element, both running along X:
    /// board A on the z = 0 line, board B on the z = 0.5 line.
    fn two_board_frame() -> AnalysisFrame {
        AnalysisFrame {
            schema_version: "FP_ANALYSIS_FRAME_1".into(),
            frame_id: "frame:test".into(),
            coordinate_basis: "PALLET_LOCAL".into(),
            nodes: vec![
                node("node:a0", 0.0, 0.1, 0.0),
                node("node:a1", 1.0, 0.1, 0.0),
                node("node:b0", 0.0, 0.1, 0.5),
                node("node:b1", 1.0, 0.1, 0.5),
            ],
            elements: vec![
                deckboard("element:0001", "node:a0", "node:a1"),
                deckboard("element:0002", "node:b0", "node:b1"),
            ],
            supports: Vec::new(),
            loads: Vec::new(),
            connectors: Vec::new(),
            constraints: Vec::new(),
            frame_hash: "frame-hash".into(),
        }
    }

    fn two_board_member_map() -> PalletMemberMap {
        PalletMemberMap {
            entries: vec![
                PalletMemberMapEntry {
                    member_id: "member:top:0001".into(),
                    kind: "TOP_DECKBOARD".into(),
                    run_axis: "X".into(),
                    segment_ids: vec!["element:0001".into()],
                    extra: Extra::new(),
                },
                PalletMemberMapEntry {
                    member_id: "member:top:0002".into(),
                    kind: "TOP_DECKBOARD".into(),
                    run_axis: "X".into(),
                    segment_ids: vec!["element:0002".into()],
                    extra: Extra::new(),
                },
            ],
            map_sha256: "member-map-hash".into(),
            extra: Extra::new(),
        }
    }

    fn patch(kind: &str, contact_id: &str, center: Vec3, force: Vec3) -> PalletTopContactPatch {
        let mut extra = Extra::new();
        extra.insert("sourceId".into(), Value::String("package:0001".into()));
        PalletTopContactPatch {
            kind: kind.into(),
            contact_id: contact_id.into(),
            center: Tagged3::point_m(center),
            force: Tagged3::polar("N", force),
            free_moment: Tagged3::axial("N_m", Vec3::ZERO),
            normal_stiffness_n_per_m: 4.0e6,
            orientation: None,
            boundary: None,
            samples: None,
            half_size_x: None,
            half_size_z: None,
            half_length: None,
            half_width: None,
            inner_radius: None,
            outer_radius: None,
            extra,
        }
    }

    fn nodal_force_sum(loads: &[FrameLoad]) -> Vec3 {
        let mut sum = Vec3::ZERO;
        for load in loads {
            if let FrameLoad::NodalForce { force, .. } = load {
                sum.x += force.x;
                sum.y += force.y;
                sum.z += force.z;
            }
        }
        sum
    }

    /// The rectangle's half-sizes and centre are exact dyadic rationals inside
    /// board A only, so the clip's centroid is exactly (0.5, 0.0) and the
    /// element's natural coordinate is exactly 0.5 — which makes the
    /// (1−t)/t force split exact in binary floating point.
    fn rectangular_patch_projection() -> PalletContactProjectionResult {
        let mut contact = patch(
            "RECTANGULAR_PATCH",
            "contact:0001",
            Vec3 { x: 0.5, y: 0.11, z: 0.0 },
            Vec3 { x: 40.0, y: -1000.0, z: 24.0 },
        );
        contact.half_size_x = Some(Quantity { unit: "m".into(), value: 0.125 });
        contact.half_size_z = Some(Quantity { unit: "m".into(), value: 0.03125 });
        contact.orientation = Some(Quantity { unit: "rad".into(), value: 0.0 });
        project_package_contacts(
            &two_board_frame(),
            &two_board_member_map(),
            &[contact],
            &profile(),
        )
        .expect("the rectangular patch projects")
    }

    #[test]
    fn rectangular_patch_conserves_the_patch_force_exactly() {
        let result = rectangular_patch_projection();
        // The horizontal half travels as nodal forces on the deck; the vertical
        // half travels as the face node's load. Their sum is the patch force,
        // and with weight = 1 and t = 0.5 it is exact, not merely within
        // tolerance.
        let deck = nodal_force_sum(&result.loads);
        let face = nodal_force_sum(&result.face_system.loads);
        assert_eq!(deck.x + face.x, 40.0);
        assert_eq!(deck.y + face.y, -1000.0);
        assert_eq!(deck.z + face.z, 24.0);
        // The vertical force is NOT on the deck and the horizontal force is NOT
        // on the face — the split by direction, asserted so a port that put the
        // whole patch on one side would fail here.
        assert_eq!(deck.y, 0.0);
        assert_eq!(face, Vec3 { x: 0.0, y: -1000.0, z: 0.0 });
    }

    #[test]
    fn rectangular_patch_audit_accepts() {
        let result = rectangular_patch_projection();
        assert!(result.audit.accepted);
        assert!(result.audit.force_residual_norm_n <= 1e-6);
        assert!(result.audit.moment_residual_norm_nm <= 1e-6);
        assert!(result.audit.resultant_location_residual_m <= 1e-6);
        assert_eq!(result.audit.input_resultant.force.y.value, -1000.0);
        assert_eq!(result.audit.projected_resultant.force.y.value, -1000.0);
    }

    #[test]
    fn rectangular_patch_builds_one_face_per_contact_and_two_springs_per_sample() {
        let result = rectangular_patch_projection();
        // One bearing sample (the patch lies wholly on board A), t = 0.5 so both
        // the start and end halves carry share > 0. The construction rules:
        //   nodes       = 1 face + 2 bearings + 2 seats
        //   constraints = 1 bearing ECCENTRIC_CONNECTION + 1 seat one, per half
        //   connectors  = 1 spring per half
        //   supports    = 1 per contact with vertical force
        //   face loads  = 1 force + 1 moment per contact
        assert_eq!(result.face_system.nodes.len(), 5);
        assert_eq!(result.face_system.constraints.len(), 4);
        assert_eq!(result.face_system.connectors.len(), 2);
        assert_eq!(result.face_system.supports.len(), 1);
        assert_eq!(result.face_system.loads.len(), 2);
        // Three loads per response point: force-start, force-end, moment.
        assert_eq!(result.loads.len(), 3);
        assert_eq!(result.contact_map.len(), 1);
        assert_eq!(result.contact_map[0].response_points.len(), 1);
        assert_eq!(result.contact_map[0].response_points[0].element_id, "element:0001");
        assert_eq!(result.contact_map[0].response_points[0].element_natural_coordinate, 0.5);
        assert_eq!(result.contact_map[0].response_points[0].normalized_contact_weight, 1.0);
        // The two springs split the patch stiffness by (1−t)/t: 4.0e6 × 1 × 0.5.
        for connector in &result.face_system.connectors {
            assert_eq!(connector.axial_stiffness.as_ref().expect("spring").value, 2.0e6);
        }
        // The bearings spread along x (0 and 1) but not along z, so rz is
        // restrained by the spread and rx is grounded.
        assert_eq!(
            result.face_system.supports[0].fixed_dofs,
            [true, false, true, true, true, false]
        );
        assert_eq!(result.face_system.nodes[0].node_id, "node:face:contact:0001");
        // Sorted canonically.
        let node_ids: Vec<&str> =
            result.face_system.nodes.iter().map(|n| n.node_id.as_str()).collect();
        let mut sorted = node_ids.clone();
        sorted.sort_by(|left, right| compare_canonical_utf8(left, right));
        assert_eq!(node_ids, sorted);
    }

    #[test]
    fn pressure_field_distributes_by_sample_weights() {
        let mut contact = patch(
            "PRESSURE_FIELD",
            "contact:0001",
            Vec3 { x: 0.5, y: 0.11, z: 0.25 },
            Vec3 { x: 40.0, y: -1000.0, z: 0.0 },
        );
        contact.boundary = Some(vec![
            Tagged3::point_m(Vec3 { x: 0.25, y: 0.11, z: 0.0 }),
            Tagged3::point_m(Vec3 { x: 0.75, y: 0.11, z: 0.0 }),
            Tagged3::point_m(Vec3 { x: 0.75, y: 0.11, z: 0.5 }),
            Tagged3::point_m(Vec3 { x: 0.25, y: 0.11, z: 0.5 }),
        ]);
        // Two samples of UNEQUAL weight: the non-uniform branch, which keeps the
        // authored quadrature instead of re-deriving weights from clipped area.
        contact.samples = Some(vec![
            ContactPressureSample {
                point: Tagged3::point_m(Vec3 { x: 0.25, y: 0.11, z: 0.0 }),
                normalized_weight: 0.25,
            },
            ContactPressureSample {
                point: Tagged3::point_m(Vec3 { x: 0.75, y: 0.11, z: 0.5 }),
                normalized_weight: 0.75,
            },
        ]);
        let result = project_package_contacts(
            &two_board_frame(),
            &two_board_member_map(),
            &[contact],
            &profile(),
        )
        .expect("the pressure field projects");
        assert!(result.audit.accepted);
        let points = &result.contact_map[0].response_points;
        assert_eq!(points.len(), 2);
        // Sample one bears on board A, sample two on board B — and each carries
        // its OWN authored weight, not the 0.5/0.5 an area re-derivation over two
        // equal boards would produce.
        assert_eq!(points[0].element_id, "element:0001");
        assert_eq!(points[0].normalized_contact_weight, 0.25);
        assert_eq!(points[0].element_natural_coordinate, 0.25);
        assert_eq!(points[1].element_id, "element:0002");
        assert_eq!(points[1].normalized_contact_weight, 0.75);
        assert_eq!(points[1].element_natural_coordinate, 0.75);
        // The horizontal 40 N splits 10 N / 30 N by those weights, and each share
        // then splits (1−t)/t over its board's endpoint nodes.
        let force_of = |load_id: &str| -> Vec3 {
            result
                .loads
                .iter()
                .find_map(|load| match load {
                    FrameLoad::NodalForce { load_id: id, force, .. } if id == load_id => {
                        Some(force.vec())
                    }
                    _ => None,
                })
                .unwrap_or_else(|| panic!("{load_id} is emitted"))
        };
        assert_eq!(force_of("load:contact:contact:0001:0000:force-start").x, 7.5);
        assert_eq!(force_of("load:contact:contact:0001:0000:force-end").x, 2.5);
        assert_eq!(force_of("load:contact:contact:0001:0001:force-start").x, 7.5);
        assert_eq!(force_of("load:contact:contact:0001:0001:force-end").x, 22.5);
        // Two samples, both with 0 < t < 1, so both halves of both samples carry
        // a spring: 1 face + 4 bearings + 4 seats.
        assert_eq!(result.face_system.nodes.len(), 9);
        assert_eq!(result.face_system.constraints.len(), 8);
        assert_eq!(result.face_system.connectors.len(), 4);
        assert_eq!(result.loads.len(), 6);
    }

    #[test]
    fn a_contact_off_the_deck_names_where_it_is_and_where_the_deck_is() {
        let contact = patch(
            "POINT",
            "contact:0001",
            Vec3 { x: 9.0, y: 0.11, z: 9.0 },
            Vec3 { x: 0.0, y: -1000.0, z: 0.0 },
        );
        let error = project_package_contacts(
            &two_board_frame(),
            &two_board_member_map(),
            &[contact],
            &profile(),
        )
        .expect_err("a point off every board has no bearing surface");
        assert_eq!(error.code, "CONTACT_HAS_NO_BEARING_SURFACE");
        assert_eq!(
            error.message,
            "CONTACT_HAS_NO_BEARING_SURFACE:contact:0001 (centre x=9.0000 z=9.0000; \
             2 top surfaces spanning x=0.0000…1.0000 z=-0.0500…0.5500)"
        );
    }
}
