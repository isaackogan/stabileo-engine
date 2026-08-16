//! `recoverPalletTopResponse` — read the solved deck back out as one
//! rigid-body motion per package contact.
//!
//! Literal port of `packages/analysis/pallet/src/top-response.ts`.
//!
//! Hashing (PORTING.md rule 7): the reference calls
//! `validatePalletTopResponseRecoveryRequest`, which re-derives the frame's
//! geometry hash and the face-system-stripped geometry hash to bind three
//! separately-transported artifacts (frame, projection, kernel result) to one
//! another. Inside this crate those artifacts are the caller's own objects, so
//! the binding checks are skipped and `responseSha256` carries the literal
//! placeholder `"internal"`; `frameHash` and `kernelResultHash` are echoed from
//! their inputs.

use std::collections::HashMap;

use crate::schema::{
    AnalysisFrame, FrameElement, KernelNodeResponse, KernelResult, NumericalAcceptanceProfile,
    PalletContactProjectionResult, PalletTopContactResponse, PalletTopResponse, Tagged3,
};
use crate::types::{PalletError, PalletResult, Vec3};

use super::compare_canonical_utf8;

pub fn recover_pallet_top_response(
    frame: &AnalysisFrame,
    projection: &PalletContactProjectionResult,
    kernel_result: &KernelResult,
    numerical_profile: &NumericalAcceptanceProfile,
) -> PalletResult<PalletTopResponse> {
    // Binding by construction: same process, same objects. (The reference's
    // `validatePalletTopResponseRecoveryRequest` re-derives and compares
    // `solvedFrameHash`, `solvedFrameGeometryHash` and the face-system-stripped
    // geometry hash against `projection.sourceFrameGeometryHash`.)
    let element_by_id: HashMap<&str, &FrameElement> =
        frame.elements.iter().map(|element| (element.element_id.as_str(), element)).collect();
    let response_by_node_id: HashMap<&str, &KernelNodeResponse> = kernel_result
        .node_responses
        .iter()
        .map(|response| (response.node_id.as_str(), response))
        .collect();
    let mut contacts: Vec<PalletTopContactResponse> =
        Vec::with_capacity(projection.contact_map.len());
    for contact in &projection.contact_map {
        let weight_sum = contact
            .response_points
            .iter()
            .fold(0.0f64, |sum, point| sum + point.normalized_contact_weight);
        if (weight_sum - 1.0).abs() > numerical_profile.geometry_tolerance_m {
            return Err(PalletError::sentence(format!(
                "PALLET_TOP_RESPONSE_WEIGHT_SUM:{}",
                contact.contact_id
            )));
        }
        let mut translation = Vec3::ZERO;
        let mut rotation = Vec3::ZERO;
        for point in &contact.response_points {
            let Some(element) = element_by_id.get(point.element_id.as_str()).copied() else {
                return Err(PalletError::sentence(format!(
                    "PALLET_TOP_RESPONSE_ELEMENT_MISSING:{}",
                    point.element_id
                )));
            };
            let start = response_by_node_id.get(element.start_node_id.as_str()).copied();
            let end = response_by_node_id.get(element.end_node_id.as_str()).copied();
            let (Some(start), Some(end)) = (start, end) else {
                return Err(PalletError::sentence(format!(
                    "PALLET_TOP_RESPONSE_NODE_RESULT_MISSING:{}",
                    point.element_id
                )));
            };
            let t = point.element_natural_coordinate;
            let interpolate = |left: f64, right: f64| left * (1.0 - t) + right * t;
            let local_rotation = Vec3 {
                x: interpolate(start.rotation.x, end.rotation.x),
                y: interpolate(start.rotation.y, end.rotation.y),
                z: interpolate(start.rotation.z, end.rotation.z),
            };
            let local_translation = Vec3 {
                x: interpolate(start.translation.x, end.translation.x),
                y: interpolate(start.translation.y, end.translation.y),
                z: interpolate(start.translation.z, end.translation.z),
            };
            // The element's axis motion carried out to the sample's own point:
            // the rigid arm turns with the section, so the point moves by
            // rotation × offset on top of the axis translation.
            let offset_translation = local_rotation.cross(point.rigid_offset_from_element_axis.vec());
            let weight = point.normalized_contact_weight;
            translation.x += (local_translation.x + offset_translation.x) * weight;
            translation.y += (local_translation.y + offset_translation.y) * weight;
            translation.z += (local_translation.z + offset_translation.z) * weight;
            rotation.x += local_rotation.x * weight;
            rotation.y += local_rotation.y * weight;
            rotation.z += local_rotation.z * weight;
        }
        contacts.push(PalletTopContactResponse {
            contact_id: contact.contact_id.clone(),
            translation: Tagged3::polar("m", translation),
            rotation: Tagged3::axial("rad", rotation),
        });
    }
    contacts.sort_by(|left, right| compare_canonical_utf8(&left.contact_id, &right.contact_id));
    Ok(PalletTopResponse {
        schema_version: "FP_PALLET_TOP_RESPONSE_1".into(),
        frame_hash: frame.frame_hash.clone(),
        kernel_result_hash: kernel_result.result_hash.clone(),
        contacts,
        // PORTING.md rule 7: the application re-derives this stamp on the value
        // it gets back; nothing inside the process consumes it.
        response_sha256: "internal".into(),
    })
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::schema::{
        ContactFaceSystem, ContactMapEntry, ContactResponsePoint, Extra, Quantity, Resultant,
        ResultantAxes, ResultantConservationAudit,
    };

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

    fn zero_resultant() -> Resultant {
        let axes = || ResultantAxes {
            x: Quantity { unit: "N".into(), value: 0.0 },
            y: Quantity { unit: "N".into(), value: 0.0 },
            z: Quantity { unit: "N".into(), value: 0.0 },
        };
        Resultant { force: axes(), moment: axes() }
    }

    fn frame() -> AnalysisFrame {
        AnalysisFrame {
            schema_version: "FP_ANALYSIS_FRAME_1".into(),
            frame_id: "frame:test".into(),
            coordinate_basis: "PALLET_LOCAL".into(),
            nodes: Vec::new(),
            elements: vec![FrameElement {
                element_id: "element:0001".into(),
                start_node_id: "node:a0".into(),
                end_node_id: "node:a1".into(),
                area_m2: 0.002,
                shear_area_y_m2: 0.002,
                shear_area_z_m2: 0.002,
                torsional_constant_m4: 1.0e-8,
                second_moment_yy_m4: 1.0e-6,
                second_moment_zz_m4: 1.0e-8,
                elastic_modulus: Quantity { unit: "Pa".into(), value: 9.0e9 },
                shear_modulus: Quantity { unit: "Pa".into(), value: 6.0e8 },
                local_y_axis: Tagged3::polar("dimensionless", Vec3 { x: 0.0, y: 1.0, z: 0.0 }),
                release_start: [false; 6],
                release_end: [false; 6],
            }],
            supports: Vec::new(),
            loads: Vec::new(),
            connectors: Vec::new(),
            constraints: Vec::new(),
            frame_hash: "frame-hash".into(),
        }
    }

    fn response_point(
        ordinal: &str,
        natural: f64,
        offset: Vec3,
        weight: f64,
    ) -> ContactResponsePoint {
        ContactResponsePoint {
            response_point_id: format!("response:contact:0001:{ordinal}"),
            element_id: "element:0001".into(),
            element_natural_coordinate: natural,
            global_point: Tagged3::point_m(Vec3::ZERO),
            rigid_offset_from_element_axis: Tagged3::polar("m", offset),
            normalized_contact_weight: weight,
        }
    }

    fn projection(response_points: Vec<ContactResponsePoint>) -> PalletContactProjectionResult {
        PalletContactProjectionResult {
            schema_version: "FP_PALLET_CONTACT_PROJECTION_RESULT_3".into(),
            loads: Vec::new(),
            face_system: ContactFaceSystem {
                nodes: Vec::new(),
                constraints: Vec::new(),
                connectors: Vec::new(),
                supports: Vec::new(),
                loads: Vec::new(),
            },
            contact_map: vec![ContactMapEntry {
                contact_id: "contact:0001".into(),
                response_points,
            }],
            audit: ResultantConservationAudit {
                input_resultant: zero_resultant(),
                projected_resultant: zero_resultant(),
                force_residual_norm_n: 0.0,
                moment_residual_norm_nm: 0.0,
                resultant_location_residual_m: 0.0,
                accepted: true,
            },
            extra: Extra::new(),
        }
    }

    fn kernel_result() -> KernelResult {
        KernelResult {
            schema_version: "FP_KERNEL_RESULT_1".into(),
            request_id: "request:test".into(),
            active_state_id: "state:test".into(),
            active_state_hash: "state-hash".into(),
            node_responses: vec![
                KernelNodeResponse {
                    node_id: "node:a0".into(),
                    translation: Tagged3::polar(
                        "m",
                        Vec3 { x: 0.001, y: -0.002, z: 0.003 },
                    ),
                    rotation: Tagged3::axial("rad", Vec3 { x: 0.01, y: 0.02, z: 0.03 }),
                },
                KernelNodeResponse {
                    node_id: "node:a1".into(),
                    translation: Tagged3::polar(
                        "m",
                        Vec3 { x: 0.005, y: -0.006, z: 0.007 },
                    ),
                    rotation: Tagged3::axial("rad", Vec3 { x: 0.04, y: 0.05, z: 0.06 }),
                },
            ],
            reactions: Vec::new(),
            element_end_forces: Vec::new(),
            connector_responses: Vec::new(),
            constraint_forces: Vec::new(),
            applied_resultant: zero_resultant(),
            reaction_resultant: zero_resultant(),
            force_residual: Tagged3::polar("N", Vec3::ZERO),
            moment_residual: Tagged3::axial("N_m", Vec3::ZERO),
            diagnostics: Vec::new(),
            result_hash: "kernel-result-hash".into(),
        }
    }

    /// Two response points at the two ends of one element, weighted 0.25/0.75,
    /// each carrying a 0.01 m arm up to the board's surface. The expected
    /// numbers below are written out longhand from the reference formula —
    /// `interpolate(l, r) = l·(1−t) + r·t`, `offset = rotation × arm`,
    /// `translation += (local + offset)·w`, `rotation += local·w` — in the
    /// reference's own evaluation order, so the assertion is not a restatement
    /// of the implementation's expression tree.
    #[test]
    fn weighted_contact_motion_matches_the_reference_formula() {
        let arm = Vec3 { x: 0.0, y: 0.01, z: 0.0 };
        let result = recover_pallet_top_response(
            &frame(),
            &projection(vec![
                response_point("0000", 0.0, arm, 0.25),
                response_point("0001", 1.0, arm, 0.75),
            ]),
            &kernel_result(),
            &profile(),
        )
        .expect("the contact recovers");
        assert_eq!(result.contacts.len(), 1);
        let contact = &result.contacts[0];
        assert_eq!(contact.contact_id, "contact:0001");
        assert_eq!(contact.translation.kind, "POLAR_VECTOR");
        assert_eq!(contact.translation.unit, "m");
        assert_eq!(contact.rotation.kind, "AXIAL_VECTOR");
        assert_eq!(contact.rotation.unit, "rad");

        // t = 0 → local = the start node's response; t = 1 → the end node's.
        // offset = rotation × (0, 0.01, 0)
        //        = (r.y·0 − r.z·0.01, r.z·0 − r.x·0, r.x·0.01 − r.y·0)
        let start_offset_x = 0.02 * 0.0 - 0.03 * 0.01;
        let start_offset_y = 0.03 * 0.0 - 0.01 * 0.0;
        let start_offset_z = 0.01 * 0.01 - 0.02 * 0.0;
        let end_offset_x = 0.05 * 0.0 - 0.06 * 0.01;
        let end_offset_y = 0.06 * 0.0 - 0.04 * 0.0;
        let end_offset_z = 0.04 * 0.01 - 0.05 * 0.0;
        let expected_translation = Vec3 {
            x: (0.001 + start_offset_x) * 0.25 + (0.005 + end_offset_x) * 0.75,
            y: (-0.002 + start_offset_y) * 0.25 + (-0.006 + end_offset_y) * 0.75,
            z: (0.003 + start_offset_z) * 0.25 + (0.007 + end_offset_z) * 0.75,
        };
        let expected_rotation = Vec3 {
            x: 0.01 * 0.25 + 0.04 * 0.75,
            y: 0.02 * 0.25 + 0.05 * 0.75,
            z: 0.03 * 0.25 + 0.06 * 0.75,
        };
        assert_eq!(contact.translation.vec(), expected_translation);
        assert_eq!(contact.rotation.vec(), expected_rotation);

        // The rigid arm actually moved the point: without the rotation × offset
        // term the x translation would be the plain weighted axis translation,
        // and it is not.
        let axis_only_x = 0.001 * 0.25 + 0.005 * 0.75;
        assert_ne!(contact.translation.x, axis_only_x);
        // Nor is it the unweighted mean — the 0.25/0.75 split is honoured.
        let mean_x = ((0.001 + start_offset_x) + (0.005 + end_offset_x)) / 2.0;
        assert_ne!(contact.translation.x, mean_x);

        assert_eq!(result.frame_hash, "frame-hash");
        assert_eq!(result.kernel_result_hash, "kernel-result-hash");
        assert_eq!(result.response_sha256, "internal");
    }

    #[test]
    fn interpolation_reads_the_element_at_its_natural_coordinate() {
        // One point at mid-span carrying the whole contact and NO rigid arm:
        // the contact motion is then the plain midpoint interpolation, which
        // pins down `interpolate` independently of the weighting.
        let result = recover_pallet_top_response(
            &frame(),
            &projection(vec![response_point("0000", 0.5, Vec3::ZERO, 1.0)]),
            &kernel_result(),
            &profile(),
        )
        .expect("the contact recovers");
        let contact = &result.contacts[0];
        assert_eq!(contact.translation.x, 0.001 * 0.5 + 0.005 * 0.5);
        assert_eq!(contact.translation.y, -0.002 * 0.5 + -0.006 * 0.5);
        assert_eq!(contact.translation.z, 0.003 * 0.5 + 0.007 * 0.5);
        assert_eq!(contact.rotation.x, 0.01 * 0.5 + 0.04 * 0.5);
    }

    #[test]
    fn weights_that_do_not_sum_to_one_are_refused() {
        let error = recover_pallet_top_response(
            &frame(),
            &projection(vec![
                response_point("0000", 0.0, Vec3::ZERO, 0.25),
                response_point("0001", 1.0, Vec3::ZERO, 0.5),
            ]),
            &kernel_result(),
            &profile(),
        )
        .expect_err("a contact whose weights do not partition unity is refused");
        assert_eq!(error.code, "PALLET_TOP_RESPONSE_WEIGHT_SUM");
        assert_eq!(error.message, "PALLET_TOP_RESPONSE_WEIGHT_SUM:contact:0001");
    }

    #[test]
    fn a_response_point_on_an_unknown_element_is_refused() {
        let mut mapped = projection(vec![response_point("0000", 0.5, Vec3::ZERO, 1.0)]);
        mapped.contact_map[0].response_points[0].element_id = "element:9999".into();
        let error =
            recover_pallet_top_response(&frame(), &mapped, &kernel_result(), &profile())
                .expect_err("an unknown element is refused");
        assert_eq!(error.code, "PALLET_TOP_RESPONSE_ELEMENT_MISSING");
        assert_eq!(error.message, "PALLET_TOP_RESPONSE_ELEMENT_MISSING:element:9999");
    }
}
