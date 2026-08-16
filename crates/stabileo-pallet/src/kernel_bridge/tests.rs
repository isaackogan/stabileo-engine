//! Kernel-bridge tests. Every solve here goes through the REAL vendored
//! kernel (`solver::linear::solve_3d`), not a stub.

use dedaliano_engine::types::{
    AnalysisResults3D, DiagnosticCode, Displacement3D, ElementForces3D, Reaction3D, Severity,
    StructuredDiagnostic,
};
use serde_json::{json, Value};

use crate::kernel_bridge::compile::compile_static_active_state;
use crate::kernel_bridge::equilibrium::audit_equilibrium;
use crate::kernel_bridge::normalize::normalize_static_result;
use crate::kernel_bridge::number_format::to_precision;
use crate::kernel_bridge::recovery::{recover_frame_response, FrameResponseStationCount};
use crate::kernel_bridge::{
    solve_static_active_state, solve_static_active_state_detailed, INTERNAL_IDENTITY,
};
use crate::schema::{AnalysisFrame, NumericalAcceptanceProfile, Quantity, Resultant, ResultantAxes};

// ---------------------------------------------------------------------------
// Fixtures
// ---------------------------------------------------------------------------

/// A 1 m cantilever along the frame's x axis: `n1` at the origin, `n2` at
/// (1, 0, 0), one element between them, one 1 kN downward nodal force on the
/// free end. `support` is spliced in so each test can choose its own.
fn frame_json(support: Value) -> Value {
    json!({
        "schemaVersion": "FP_ANALYSIS_FRAME_1",
        "frameId": "frame-under-test",
        "coordinateBasis": "PALLET_FRAME",
        "nodes": [
            { "nodeId": "n1", "position": { "kind": "POINT", "unit": "m", "x": 0.0, "y": 0.0, "z": 0.0 } },
            { "nodeId": "n2", "position": { "kind": "POINT", "unit": "m", "x": 1.0, "y": 0.0, "z": 0.0 } }
        ],
        "elements": [{
            "elementId": "e1",
            "startNodeId": "n1",
            "endNodeId": "n2",
            "areaM2": 0.01,
            "shearAreaYM2": 0.008_333_333_333_333_333,
            "shearAreaZM2": 0.008_333_333_333_333_333,
            "torsionalConstantM4": 1.4e-6,
            "secondMomentYyM4": 8.333_333_333_333_333e-7,
            "secondMomentZzM4": 8.333_333_333_333_333e-7,
            "elasticModulus": { "unit": "Pa", "value": 1.0e10 },
            "shearModulus": { "unit": "Pa", "value": 6.0e8 },
            "localYAxis": { "kind": "POLAR_VECTOR", "unit": "m", "x": 0.0, "y": 1.0, "z": 0.0 },
            "releaseStart": [false, false, false, false, false, false],
            "releaseEnd": [false, false, false, false, false, false]
        }],
        "supports": [support],
        "loads": [{
            "kind": "NODAL_FORCE",
            "loadId": "l1",
            "nodeId": "n2",
            "force": { "kind": "POLAR_VECTOR", "unit": "N", "x": 0.0, "y": -1000.0, "z": 0.0 },
            "application": null
        }],
        "connectors": [],
        "constraints": [],
        "frameHash": "internal"
    })
}

/// Fixed on every axis but the vertical, which is a 1e8 N/m spring — the
/// "fixed AND elastic" support, on axes that do not collide.
fn fixed_and_elastic_support() -> Value {
    json!({
        "supportId": "s1",
        "nodeId": "n1",
        "active": true,
        "fixedDofs": [true, false, true, true, true, true],
        "prescribedTranslations": null,
        "prescribedRotations": null,
        "elasticStiffness": { "kind": "POLAR_VECTOR", "unit": "N_per_m", "x": 0.0, "y": 1.0e8, "z": 0.0 }
    })
}

/// Fully fixed, and pushed 1 mm down: the prescribed-displacement case.
fn prescribed_displacement_support() -> Value {
    json!({
        "supportId": "s1",
        "nodeId": "n1",
        "active": true,
        "fixedDofs": [true, true, true, true, true, true],
        "prescribedTranslations": { "kind": "POLAR_VECTOR", "unit": "m", "x": 0.0, "y": -0.001, "z": 0.0 },
        "prescribedRotations": null,
        "elasticStiffness": null
    })
}

fn frame(support: Value) -> AnalysisFrame {
    serde_json::from_value(frame_json(support)).expect("the fixture frame parses")
}

fn profile() -> NumericalAcceptanceProfile {
    serde_json::from_value(json!({
        "schemaVersion": "FP_NUMERICAL_ACCEPTANCE_PROFILE_1",
        "profileId": "profile-under-test",
        "profileSha256": "internal",
        "geometryToleranceM": 1.0e-6,
        "forceToleranceN": 0.001,
        "momentToleranceNm": 0.001,
        "lengthToleranceM": 1.0e-6,
        "complementarityToleranceN": 0.001,
        "coupledIterationLimit": 32,
        "coupledTranslationToleranceM": 1.0e-6,
        "coupledRotationToleranceRad": 1.0e-6,
        "coupledLoadShareTolerance": 0.001
    }))
    .expect("the fixture profile parses")
}

// ---------------------------------------------------------------------------
// (a) A real solve through the vendored kernel, audited
// ---------------------------------------------------------------------------

#[test]
fn solves_a_two_node_frame_through_the_real_kernel_and_balances() {
    let result =
        solve_static_active_state(&frame(fixed_and_elastic_support()), &profile(), "req-1", "state-1")
            .expect("the solve is accepted by the equilibrium audit");

    assert_eq!(result.schema_version, "FP_KERNEL_RESULT_1");
    assert_eq!(result.request_id, "req-1");
    assert_eq!(result.active_state_id, "state-1");
    assert_eq!(result.active_state_hash, INTERNAL_IDENTITY);
    assert_eq!(result.result_hash, INTERNAL_IDENTITY);

    // The applied resultant is the 1 kN pushing DOWN, with its moment about
    // the origin: r x F = (1,0,0) x (0,-1000,0) = (0, 0, -1000).
    assert_eq!(result.applied_resultant.force.y.value, -1000.0);
    assert_eq!(result.applied_resultant.moment.z.value, -1000.0);

    // ONE reaction, at the one active support, and it is the applied force
    // with the opposite sign — the TS convention is that applied and reacted
    // SUM to zero, they do not difference.
    assert_eq!(result.reactions.len(), 1);
    let reaction = &result.reactions[0];
    assert_eq!(reaction.support_id, "s1");
    assert_eq!(reaction.force.unit, "N");
    assert!(
        (reaction.force.y - 1000.0).abs() < 1e-6,
        "reaction carries the load: {:?}",
        reaction.force
    );
    assert!(reaction.force.x.abs() < 1e-6);
    assert!(reaction.force.z.abs() < 1e-6);

    // The audit accepted, so every residual component is at solver noise.
    for component in [
        result.force_residual.x,
        result.force_residual.y,
        result.force_residual.z,
    ] {
        assert!(component.abs() < 1e-6, "force residual component {component}");
    }
    for component in [
        result.moment_residual.x,
        result.moment_residual.y,
        result.moment_residual.z,
    ] {
        assert!(component.abs() < 1e-6, "moment residual component {component}");
    }

    // Both node responses came back, named, sorted.
    assert_eq!(
        result.node_responses.iter().map(|node| node.node_id.as_str()).collect::<Vec<&str>>(),
        vec!["n1", "n2"]
    );
    // The spring at n1 settles by F/k = 1000/1e8 = 1e-5 m, downwards.
    let n1 = &result.node_responses[0];
    assert!(
        (n1.translation.y - (-1e-5)).abs() < 1e-9,
        "spring settlement: {:?}",
        n1.translation
    );
    assert_eq!(result.element_end_forces.len(), 1);
    assert_eq!(result.element_end_forces[0].element_id, "e1");
}

// ---------------------------------------------------------------------------
// (b) A prescribed-displacement support
// ---------------------------------------------------------------------------

#[test]
fn solves_a_prescribed_displacement_support() {
    let result = solve_static_active_state(
        &frame(prescribed_displacement_support()),
        &profile(),
        "req-2",
        "state-2",
    )
    .expect("the prescribed-displacement solve is accepted");

    // The support node sits exactly where it was told to sit: 1 mm down.
    let n1 = result
        .node_responses
        .iter()
        .find(|node| node.node_id == "n1")
        .expect("n1 is in the response");
    assert!(
        (n1.translation.y - (-0.001)).abs() < 1e-12,
        "prescribed settlement: {:?}",
        n1.translation
    );
    assert!(n1.translation.x.abs() < 1e-12 && n1.translation.z.abs() < 1e-12);

    // A rigid base under a determinate cantilever: the reaction is still the
    // whole load, and the audit still accepts.
    assert_eq!(result.reactions.len(), 1);
    let reaction = &result.reactions[0];
    assert!(
        (reaction.force.y - 1000.0).abs() < 1e-6,
        "reaction carries the load: {:?}",
        reaction.force
    );
    // The fixed base also supplies the restraining couple: -1000 N·m applied
    // about z, +1000 N·m reacted.
    assert!(
        (reaction.moment.z - 1000.0).abs() < 1e-6,
        "reaction couple: {:?}",
        reaction.moment
    );
    assert!(result.force_residual.y.abs() < 1e-6);
    assert!(result.moment_residual.z.abs() < 1e-6);
}

// ---------------------------------------------------------------------------
// (c) Station recovery
// ---------------------------------------------------------------------------

#[test]
fn recovers_the_requested_stations_with_end_forces_at_the_ends() {
    let solve = solve_static_active_state_detailed(
        &frame(prescribed_displacement_support()),
        &profile(),
        "req-3",
        "state-3",
    )
    .expect("the solve is accepted");

    let recovered = recover_frame_response(
        &solve.compiled,
        &solve.raw,
        &[FrameResponseStationCount { element_id: "e1".to_string(), count: 5 }],
    )
    .expect("recovery succeeds");

    assert_eq!(recovered.schema_version, "FP_FRAME_RESPONSE_RECOVERY_RESULT_1");
    assert_eq!(recovered.request_id, "req-3");
    assert_eq!(recovered.response_hash, INTERNAL_IDENTITY);
    assert_eq!(recovered.elements.len(), 1);
    let element = &recovered.elements[0];
    assert_eq!(element.element_id, "e1");
    assert_eq!(element.stations.len(), 5);
    assert_eq!(
        element.stations.iter().map(|station| station.normalized_position).collect::<Vec<f64>>(),
        vec![0.0, 0.25, 0.5, 0.75, 1.0]
    );

    // The stations are the kernel's own element forces, in the app's units:
    // kilonewtons became newtons and kilonewton-metres became newton-metres.
    let raw_forces = &solve.raw.element_forces[0];
    let start = &element.stations[0];
    assert!((start.axial_force_n - raw_forces.n_start * 1000.0).abs() < 1e-9);
    assert!((start.shear_y_n - raw_forces.vy_start * 1000.0).abs() < 1e-9);
    assert!((start.shear_z_n - raw_forces.vz_start * 1000.0).abs() < 1e-9);
    assert!((start.torsion_nm - raw_forces.mx_start * 1000.0).abs() < 1e-9);
    assert!((start.bending_y_nm - raw_forces.my_start * 1000.0).abs() < 1e-9);
    assert!((start.bending_z_nm - raw_forces.mz_start * 1000.0).abs() < 1e-9);

    let end = &element.stations[4];
    assert!((end.axial_force_n - raw_forces.n_end * 1000.0).abs() < 1e-9);
    assert!((end.torsion_nm - raw_forces.mx_end * 1000.0).abs() < 1e-9);
    // The bending diagram at the far end is the start moment carried out over
    // the member's own length by the start shear — the kernel's convention,
    // read back through this port's unit conversion.
    let length = raw_forces.length;
    assert!(
        (end.bending_z_nm - (raw_forces.mz_start - raw_forces.vy_start * length) * 1000.0).abs()
            < 1e-6,
        "momentZ at the far end: {}",
        end.bending_z_nm
    );
    assert!(
        (end.bending_y_nm - (raw_forces.my_start + raw_forces.vz_start * length) * 1000.0).abs()
            < 1e-6
    );

    // A 1 kN tip load on a 1 m cantilever: 1000 N·m at the root, nothing at
    // the tip.
    assert!((start.bending_z_nm.abs() - 1000.0).abs() < 1e-6);
    assert!(end.bending_z_nm.abs() < 1e-6);
}

#[test]
fn recovery_refuses_a_repeated_element() {
    let solve = solve_static_active_state_detailed(
        &frame(prescribed_displacement_support()),
        &profile(),
        "req-4",
        "state-4",
    )
    .expect("the solve is accepted");
    let error = recover_frame_response(
        &solve.compiled,
        &solve.raw,
        &[
            FrameResponseStationCount { element_id: "e1".to_string(), count: 3 },
            FrameResponseStationCount { element_id: "e1".to_string(), count: 3 },
        ],
    )
    .expect_err("a repeated element is refused");
    assert_eq!(error.code, "RECOVERY_DUPLICATE_ELEMENT");
    assert_eq!(error.message, "RECOVERY_DUPLICATE_ELEMENT: e1");
}

// ---------------------------------------------------------------------------
// (d) The audit refuses a doctored result, in the TS's exact words
// ---------------------------------------------------------------------------

fn resultant(force: [f64; 3], moment: [f64; 3]) -> Resultant {
    let newton = |value: f64| Quantity { unit: "N".to_string(), value };
    let newton_metre = |value: f64| Quantity { unit: "N_m".to_string(), value };
    Resultant {
        force: ResultantAxes {
            x: newton(force[0]),
            y: newton(force[1]),
            z: newton(force[2]),
        },
        moment: ResultantAxes {
            x: newton_metre(moment[0]),
            y: newton_metre(moment[1]),
            z: newton_metre(moment[2]),
        },
    }
}

#[test]
fn the_audit_refuses_an_imbalanced_resultant() {
    // Half the load reacted: 500 N unaccounted for against a 1 N tolerance.
    let audit = audit_equilibrium(
        &resultant([0.0, -1000.0, 0.0], [0.0, 0.0, -1000.0]),
        &resultant([0.0, 500.0, 0.0], [0.0, 0.0, 1000.0]),
        1.0,
    )
    .expect("a 1 m characteristic length is legal");
    assert!(!audit.accepted);
    assert_eq!(audit.force_residual_norm, 500.0);
    assert_eq!(audit.moment_residual_norm, 0.0);
    assert_eq!(audit.force_tolerance, 1.0);
    assert_eq!(audit.moment_tolerance, 1.0);
    assert_eq!(audit.force_residual.y, -500.0);
    assert_eq!(audit.force_residual.unit, "N");
    assert_eq!(audit.moment_residual.unit, "N_m");

    // And a balanced one is accepted, so the assertion above is not vacuous.
    let balanced = audit_equilibrium(
        &resultant([0.0, -1000.0, 0.0], [0.0, 0.0, -1000.0]),
        &resultant([0.0, 1000.0, 0.0], [0.0, 0.0, 1000.0]),
        1.0,
    )
    .expect("a 1 m characteristic length is legal");
    assert!(balanced.accepted);
}

#[test]
fn the_audit_refuses_a_negative_characteristic_length() {
    let error = audit_equilibrium(
        &resultant([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
        &resultant([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
        -1.0,
    )
    .expect_err("a negative characteristic length is refused");
    assert_eq!(error.message, "EQUILIBRIUM_AUDIT_CHARACTERISTIC_LENGTH:-1");
}

/// A hand-built kernel answer for the cantilever fixture, so every number in
/// the refusal sentence is chosen rather than solved for.
///
/// The kernel's basis is the frame's rotated by (x, y, z) -> (x, -z, y), so a
/// reaction of +500 N along the frame's UP axis is `fz = 0.5` kN here, and a
/// +1000 N·m couple about the frame's z is `my = -1.0` kN·m.
fn doctored_results(structured_diagnostics: Vec<StructuredDiagnostic>) -> AnalysisResults3D {
    let zero_displacement = |node_id: usize| Displacement3D {
        node_id,
        ux: 0.0,
        uy: 0.0,
        uz: 0.0,
        rx: 0.0,
        ry: 0.0,
        rz: 0.0,
        warping: None,
    };
    AnalysisResults3D {
        displacements: vec![zero_displacement(1), zero_displacement(2)],
        reactions: vec![Reaction3D {
            node_id: 1,
            fx: 0.0,
            fy: 0.0,
            fz: 0.5,
            mx: 0.0,
            my: -1.0,
            mz: 0.0,
            bimoment: None,
        }],
        element_forces: vec![ElementForces3D {
            element_id: 1,
            length: 1.0,
            n_start: 0.0,
            n_end: 0.0,
            vy_start: 0.0,
            vy_end: 0.0,
            vz_start: 0.0,
            vz_end: 0.0,
            mx_start: 0.0,
            mx_end: 0.0,
            my_start: 0.0,
            my_end: 0.0,
            mz_start: 0.0,
            mz_end: 0.0,
            release_my_start: false,
            release_my_end: false,
            release_mz_start: false,
            release_mz_end: false,
            release_t_start: false,
            release_t_end: false,
            q_yi: 0.0,
            q_yj: 0.0,
            distributed_loads_y: vec![],
            point_loads_y: vec![],
            q_zi: 0.0,
            q_zj: 0.0,
            distributed_loads_z: vec![],
            point_loads_z: vec![],
            bimoment_start: None,
            bimoment_end: None,
        }],
        plate_stresses: vec![],
        quad_stresses: vec![],
        quad_nodal_stresses: vec![],
        constraint_forces: vec![],
        diagnostics: vec![],
        solver_diagnostics: vec![],
        timings: None,
        structured_diagnostics,
        equilibrium: None,
        result_summary: None,
        solver_run_meta: None,
    }
}

#[test]
fn normalize_refuses_a_doctored_result_in_the_reference_sentence() {
    let frame = frame(prescribed_displacement_support());
    let compiled =
        compile_static_active_state(&frame, "req-5", "state-5", INTERNAL_IDENTITY).expect("compiles");
    let error = normalize_static_result(&compiled, &doctored_results(vec![]))
        .expect_err("half a reaction is not equilibrium");

    // Built by hand from `normalize.ts`, with the numbers this fixture makes:
    // a 500 N force residual against a 1 N tolerance (0.001 x 1000 N applied),
    // a moment that does balance, over a 1 m frame, one support, and a kernel
    // that said nothing.
    let expected = "CORRUPT_OUTPUT: equilibrium residual exceeds the frozen numerical profile \
                    (force 500.0 N against 1.000 N on 1000.00 N applied / 500.000 N reacted; \
                    moment 0.000 N·m against 1.000 N·m on 1000.00 / 1000.00 N·m \
                    (residual by axis 0.000, 0.000, 0.000 N·m — the two resultants SUM to zero \
                    at equilibrium, they do not difference) over a 1.000 m frame; 1 supports; \
                    kernel says [nothing])";
    assert_eq!(error.message, expected);
    assert_eq!(error.code, "CORRUPT_OUTPUT");
}

#[test]
fn the_refusal_repeats_what_the_kernel_said() {
    let frame = frame(prescribed_displacement_support());
    let compiled =
        compile_static_active_state(&frame, "req-6", "state-6", INTERNAL_IDENTITY).expect("compiles");
    let diagnostic = StructuredDiagnostic {
        code: DiagnosticCode::OverConstrainedDof,
        severity: Severity::Warning,
        message: "ignored — the sentence quotes the code, not the prose".to_string(),
        element_ids: vec![],
        node_ids: vec![1],
        dof_indices: vec![960, 961],
        phase: None,
        value: None,
        threshold: None,
    };
    let error = normalize_static_result(&compiled, &doctored_results(vec![diagnostic]))
        .expect_err("half a reaction is not equilibrium");
    assert!(
        error.message.ends_with("kernel says [warning:over_constrained_dofx1 at n1])"),
        "{}",
        error.message
    );
}

// ---------------------------------------------------------------------------
// The formatter the sentence is made of
// ---------------------------------------------------------------------------

#[test]
fn to_precision_matches_the_ecmascript_algorithm() {
    // Every expectation below is `Number.prototype.toPrecision` in node.
    assert_eq!(to_precision(0.0, 4), "0.000");
    assert_eq!(to_precision(0.0, 6), "0.00000");
    assert_eq!(to_precision(1234.5, 4), "1235"); // a tie rounds to the larger m
    assert_eq!(to_precision(1234.5, 6), "1234.50");
    assert_eq!(to_precision(1.25, 4), "1.250");
    assert_eq!(to_precision(0.001, 4), "0.001000");
    assert_eq!(to_precision(1e-7, 4), "1.000e-7");
    assert_eq!(to_precision(12345.678, 4), "1.235e+4");
    assert_eq!(to_precision(12345.678, 6), "12345.7");
    assert_eq!(to_precision(9.9999, 4), "10.00");
    assert_eq!(to_precision(1e21, 4), "1.000e+21");
    assert_eq!(to_precision(-3.5, 4), "-3.500");
    assert_eq!(to_precision(0.1 + 0.2, 6), "0.300000");
    assert_eq!(to_precision(1e-6, 4), "0.000001000");
    assert_eq!(to_precision(123456789.0, 6), "1.23457e+8");
    assert_eq!(to_precision(0.0001999, 4), "0.0001999");
    assert_eq!(to_precision(1.005, 4), "1.005");
    assert_eq!(to_precision(f64::NAN, 4), "NaN");
    assert_eq!(to_precision(f64::INFINITY, 4), "Infinity");
}

// ---------------------------------------------------------------------------
// Compile-time refusals the sentences of which are contract
// ---------------------------------------------------------------------------

#[test]
fn a_fixed_axis_may_not_also_be_elastic() {
    let support = json!({
        "supportId": "s1",
        "nodeId": "n1",
        "active": true,
        "fixedDofs": [true, true, true, true, true, true],
        "prescribedTranslations": null,
        "prescribedRotations": null,
        "elasticStiffness": { "kind": "POLAR_VECTOR", "unit": "N_per_m", "x": 0.0, "y": 1.0e8, "z": 0.0 }
    });
    let error = solve_static_active_state(&frame(support), &profile(), "req-7", "state-7")
        .expect_err("a fixed axis may not also be elastic");
    assert_eq!(error.code, "MODEL_UNSUPPORTED");
    assert_eq!(error.message, "MODEL_UNSUPPORTED: support s1 is both fixed and elastic on TY");
}

#[test]
fn an_implausible_modulus_ratio_names_the_member_and_the_band() {
    let mut frame_value = frame_json(prescribed_displacement_support());
    frame_value["elements"][0]["elementId"] = json!("member-7/segment/0-1");
    frame_value["elements"][0]["shearModulus"]["value"] = json!(1.0e8);
    let frame: AnalysisFrame = serde_json::from_value(frame_value).expect("parses");
    let error = solve_static_active_state(&frame, &profile(), "req-8", "state-8")
        .expect_err("E/G = 100 is outside the band");
    assert_eq!(
        error.message,
        "MODEL_UNSUPPORTED: implausible modulus ratio for member-7 \
         (element member-7/segment/0-1): E/G = 100 (E = 10.0 GPa, G = 0.100 GPa), \
         outside the supported band 2-30"
    );
}
