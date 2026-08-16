//! Literal port of `packages/analysis/stabileo/src/compile.ts`: an
//! `AnalysisFrame` becomes the kernel's `SolverInput3D` plus its constraint
//! list, with the id map and the local triads the normalizer needs to read the
//! answer back.
//!
//! The TS emits JSON for the WASM boundary to deserialize; this port builds
//! the same structs directly, so every field `compile.ts` writes is written
//! here into the corresponding struct field and nothing serializes.

use std::collections::HashMap;

use dedaliano_engine::types::{
    ConnectorElement, Constraint, EccentricConnectionConstraint, EqualDOFConstraint,
    LinearMPCConstraint, MPCTerm, RigidLinkConstraint, SolverDistributedLoad3D, SolverElement3D,
    SolverInput3D, SolverLoad3D, SolverMaterial, SolverNodalLoad3D, SolverNode3D, SolverPointLoad3D,
    SolverSection3D, SolverSupport3D,
};
use serde::Deserialize;
use serde_json::Value;

use crate::kernel_bridge::coordinates::{
    to_stabileo_axial, to_stabileo_local_triad, to_stabileo_point, to_stabileo_polar,
    StabileoLocalTriad,
};
use crate::kernel_bridge::id_map::NumericIdMap;
use crate::kernel_bridge::number_format::{js_number_to_string, to_precision};
use crate::kernel_bridge::units::{
    solver_force_value_to_sdk, solver_modulus_value_to_sdk, solver_moment_value_to_sdk,
    solver_stiffness_value_to_sdk,
};
use crate::schema::{AnalysisFrame, FrameElement, FrameLoad, Quantity, Resultant, ResultantAxes, Tagged3};
use crate::types::{PalletError, PalletResult, Vec3};

/// The capability set this bridge answers for. The TS gate reads the
/// `requiredCapabilities` off the kernel request and refuses anything outside
/// this set with `CAPABILITY_MISSING`.
///
/// PORT-QUESTION: the native entry point takes a frame, a profile and two ids
/// — there is no `KernelRequest`, so there is no `requiredCapabilities` array
/// to gate. The list and the check are ported and public so the coupled loop
/// can still run the gate if it ever carries a capability list, but
/// `solve_static_active_state` cannot call it with anything.
pub const SUPPORTED_CAPABILITIES: &[&str] = &[
    "FRAME_LINEAR_3D",
    "CONSTRAINTS_AND_CONNECTORS_3D",
    "REACTIONS_AND_EQUILIBRIUM_3D",
    "PRESCRIBED_DISPLACEMENT_3D",
    "EQUIVALENT_SELF_EQUILIBRATING_PRETENSION_LOAD_3D",
    "FRAME_RESPONSE_RECOVERY_3D",
];

pub fn require_capabilities(required: &[String]) -> PalletResult<()> {
    for capability in required {
        if !SUPPORTED_CAPABILITIES.contains(&capability.as_str()) {
            return Err(PalletError::sentence(format!("CAPABILITY_MISSING: {capability}")));
        }
    }
    Ok(())
}

const TOLERANCE: f64 = 1e-10;

/// TS `STABILEO_RUNTIME_IDENTITY.sdkVersion`, quoted by three
/// `MODEL_UNSUPPORTED` sentences ("… is not representable by Stabileo 0.1.2").
///
/// PORT-QUESTION: natively there is no SDK PACKAGE — the kernel is linked in
/// process, and `manifest.ts` is the app's record of the npm artifact it
/// loaded. The version is frozen here because the sentence is diagnostic
/// contract; it must be re-pinned when the vendored kernel is re-vendored.
pub const STABILEO_SDK_VERSION: &str = "0.1.2";

#[derive(Debug, Clone)]
pub struct StableIds {
    pub nodes: NumericIdMap,
    pub elements: NumericIdMap,
    pub materials: NumericIdMap,
    pub sections: NumericIdMap,
    pub supports: NumericIdMap,
    pub connectors: NumericIdMap,
    pub constraints: NumericIdMap,
}

#[derive(Debug, Clone)]
pub struct CompiledStabileoModel {
    /// TS `request.frame`; the kernel request itself is not a native type, so
    /// the two identity fields it carried travel beside the frame.
    pub frame: AnalysisFrame,
    pub request_id: String,
    pub input: SolverInput3D,
    /// The kernel constraint list.
    ///
    /// The TS builds a hand-written wire object here and casts it onto the
    /// SDK's `Constraint` type, because the shipped `.d.ts` declares an
    /// ADJACENTLY tagged union (`{ type, value }`) while the binary
    /// deserializes an INTERNALLY tagged one — every constrained solve failed
    /// with `Parse error: missing field 'masterNode'` until that was
    /// corrected, and `frame-builder` emits an `ECCENTRIC_CONNECTION` for
    /// every multi-segment member, so no real pallet frame had ever reached
    /// the kernel. Natively there is no wire and no declaration to be wrong
    /// about: this is the kernel's own enum, built by the kernel's own type.
    pub constraints: Vec<Constraint>,
    /// The frame's own constraints, parsed once: `normalize.rs` attributes
    /// constraint forces against them and the TS reads them straight off the
    /// zod-parsed frame.
    pub parsed_constraints: Vec<ParsedFrameConstraint>,
    pub ids: StableIds,
    pub element_triads: HashMap<String, StabileoLocalTriad>,
    pub applied_resultant: Resultant,
    pub active_state_id: String,
    pub active_state_hash: String,
}

fn unsupported<T>(message: String) -> PalletResult<T> {
    Err(PalletError::sentence(format!("MODEL_UNSUPPORTED: {message}")))
}

fn finite(value: f64, label: &str) -> PalletResult<f64> {
    if !value.is_finite() {
        return Err(PalletError::sentence(format!("INPUT_INVALID: {label} must be finite")));
    }
    Ok(if value == 0.0 { 0.0 } else { value })
}

/// PORT-QUESTION: the TS reaches through `!` non-null assertions for every
/// frame cross-reference (`nodes.get(element.startNodeId)!`,
/// `frame.elements.find(...)!`, `triads.get(elementId)!`), because
/// `AnalysisFrameSchema`'s `superRefine` has already refused a frame whose
/// loads, elements, supports, connectors or constraints name an id that is not
/// in the frame. A violated assertion is a bare JS `TypeError` with no ported
/// sentence, so the port refuses with `INPUT_INVALID` and names the reference.
fn invariant<T>(value: Option<T>, detail: impl FnOnce() -> String) -> PalletResult<T> {
    value.ok_or_else(|| PalletError::sentence(format!("INPUT_INVALID: {}", detail())))
}

fn dot(a: Vec3, b: Vec3) -> f64 {
    a.x * b.x + a.y * b.y + a.z * b.z
}

fn subtract(a: Vec3, b: Vec3) -> Vec3 {
    Vec3 { x: a.x - b.x, y: a.y - b.y, z: a.z - b.z }
}

fn cross(a: Vec3, b: Vec3) -> Vec3 {
    Vec3 { x: a.y * b.z - a.z * b.y, y: a.z * b.x - a.x * b.z, z: a.x * b.y - a.y * b.x }
}

fn scale(a: Vec3, factor: f64) -> Vec3 {
    Vec3 { x: a.x * factor, y: a.y * factor, z: a.z * factor }
}

fn add(a: Vec3, b: Vec3) -> Vec3 {
    Vec3 { x: a.x + b.x, y: a.y + b.y, z: a.z + b.z }
}

fn norm(a: Vec3) -> f64 {
    a.hypot3()
}

fn sdk_dof(dof: &str) -> PalletResult<usize> {
    match dof {
        "TX" => Ok(0),
        "TY" => Ok(2),
        "TZ" => Ok(1),
        "RX" => Ok(3),
        "RY" => Ok(5),
        "RZ" => Ok(4),
        // PORT-QUESTION: the TS indexes a literal object, so an unrecognised
        // DOF yields `undefined` and reaches the kernel as `NaN`. The frame
        // schema's enum makes that unreachable; the port refuses instead of
        // compiling a NaN degree of freedom.
        other => Err(PalletError::sentence(format!("INPUT_INVALID: unknown frame DOF {other}"))),
    }
}

// ---------------------------------------------------------------------------
// Frame constraints
//
// `schema.rs` carries a frame constraint as a raw `Value` (the loop never
// mutates one), so the bridge reads the app's four constraint shapes here.
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Deserialize)]
pub struct ReleaseState {
    pub tx: bool,
    pub ty: bool,
    pub tz: bool,
    pub rx: bool,
    pub ry: bool,
    pub rz: bool,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct LinearMpcTerm {
    pub node_id: String,
    pub dof: String,
    pub coefficient: f64,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "kind")]
pub enum ParsedFrameConstraint {
    #[serde(rename = "RIGID_LINK", rename_all = "camelCase")]
    RigidLink {
        constraint_id: String,
        master_node_id: String,
        slave_node_id: String,
        dofs: Vec<String>,
    },
    #[serde(rename = "ECCENTRIC_CONNECTION", rename_all = "camelCase")]
    EccentricConnection {
        constraint_id: String,
        master_node_id: String,
        slave_node_id: String,
        polar_offset: Tagged3,
        releases: ReleaseState,
    },
    #[serde(rename = "EQUAL_DOF", rename_all = "camelCase")]
    EqualDof {
        constraint_id: String,
        master_node_id: String,
        slave_node_id: String,
        dofs: Vec<String>,
    },
    #[serde(rename = "LINEAR_MPC", rename_all = "camelCase")]
    LinearMpc { constraint_id: String, terms: Vec<LinearMpcTerm> },
}

impl ParsedFrameConstraint {
    pub fn constraint_id(&self) -> &str {
        match self {
            ParsedFrameConstraint::RigidLink { constraint_id, .. }
            | ParsedFrameConstraint::EccentricConnection { constraint_id, .. }
            | ParsedFrameConstraint::EqualDof { constraint_id, .. }
            | ParsedFrameConstraint::LinearMpc { constraint_id, .. } => constraint_id,
        }
    }
}

/// PORT-QUESTION: the TS receives constraints already parsed by zod, so a
/// malformed one cannot reach `compileConstraints`. Here they arrive as raw
/// JSON and the failure has no ported sentence; `INPUT_INVALID` keeps it in
/// the family the application's kernel wrapper re-throws unwrapped.
pub fn parse_frame_constraints(
    constraints: &[Value],
) -> PalletResult<Vec<ParsedFrameConstraint>> {
    constraints
        .iter()
        .enumerate()
        .map(|(index, raw)| {
            serde_json::from_value::<ParsedFrameConstraint>(raw.clone()).map_err(|error| {
                PalletError::sentence(format!(
                    "INPUT_INVALID: frame constraint {index} does not match the frame constraint schema ({error})"
                ))
            })
        })
        .collect()
}

fn ids_for(
    frame: &AnalysisFrame,
    parsed_constraints: &[ParsedFrameConstraint],
) -> PalletResult<StableIds> {
    let material_ids: Vec<String> =
        frame.elements.iter().map(|element| format!("material:{}", element.element_id)).collect();
    let section_ids: Vec<String> =
        frame.elements.iter().map(|element| format!("section:{}", element.element_id)).collect();
    let constraint_ids: Vec<String> =
        parsed_constraints.iter().map(|constraint| constraint.constraint_id().to_string()).collect();
    Ok(StableIds {
        nodes: NumericIdMap::build(
            &frame.nodes.iter().map(|node| node.node_id.clone()).collect::<Vec<String>>(),
        )?,
        elements: NumericIdMap::build(
            &frame.elements.iter().map(|element| element.element_id.clone()).collect::<Vec<String>>(),
        )?,
        materials: NumericIdMap::build(&material_ids)?,
        sections: NumericIdMap::build(&section_ids)?,
        supports: NumericIdMap::build(
            &frame.supports.iter().map(|support| support.support_id.clone()).collect::<Vec<String>>(),
        )?,
        connectors: NumericIdMap::build(
            &frame
                .connectors
                .iter()
                .map(|connector| connector.connector_id.clone())
                .collect::<Vec<String>>(),
        )?,
        constraints: NumericIdMap::build(&constraint_ids)?,
    })
}

fn build_triads(frame: &AnalysisFrame) -> PalletResult<HashMap<String, StabileoLocalTriad>> {
    let nodes: HashMap<&str, Vec3> =
        frame.nodes.iter().map(|node| (node.node_id.as_str(), node.position.vec())).collect();
    let mut triads = HashMap::with_capacity(frame.elements.len());
    for element in &frame.elements {
        let start = *invariant(nodes.get(element.start_node_id.as_str()), || {
            format!("element {} names an unknown start node {}", element.element_id, element.start_node_id)
        })?;
        let end = *invariant(nodes.get(element.end_node_id.as_str()), || {
            format!("element {} names an unknown end node {}", element.element_id, element.end_node_id)
        })?;
        let axis = subtract(end, start);
        if norm(axis) <= TOLERANCE {
            return unsupported(format!("zero-length frame element {}", element.element_id));
        }
        triads.insert(
            element.element_id.clone(),
            to_stabileo_local_triad(axis, element.local_y_axis.vec())?,
        );
    }
    Ok(triads)
}

fn compile_nodes(
    frame: &AnalysisFrame,
    ids: &StableIds,
) -> PalletResult<HashMap<String, SolverNode3D>> {
    let mut nodes = HashMap::with_capacity(frame.nodes.len());
    for node in &frame.nodes {
        let id = ids.nodes.numeric(&node.node_id)?;
        let point = to_stabileo_point(node.position.vec());
        nodes.insert(
            id.to_string(),
            SolverNode3D {
                id,
                x: finite(point.x, "node x")?,
                y: finite(point.y, "node y")?,
                z: finite(point.z, "node z")?,
            },
        );
    }
    Ok(nodes)
}

/// The SDK takes no shear modulus: it reconstructs G = E / (2(1+ν)) and rejects
/// ν outside (-1, 0.5) inside the WASM. Wood is orthotropic — this codebase
/// freezes G = 0.069·E, i.e. ν ≈ 6.25 — so the member's real G is not
/// representable as a Poisson ratio at all. We therefore pin a legal ν and
/// preserve the PHYSICS instead of the encoding: every frame quantity the SDK
/// multiplies by G (the Timoshenko shear areas Asy/Asz and the torsional
/// constant J) is pre-scaled by G_real / G_encoded, so the products
/// G_encoded·As ≡ G_real·As_real and G_encoded·J ≡ G_real·J_real hold identically
/// up to floating-point round-off.
/// Frame elements use ν through G only, so nothing else is distorted; a future
/// plate/shell element, which consumes ν directly, would need its own encoding.
const ENCODED_POISSON_RATIO: f64 = 0.3;

/// The physically meaningful band for E/G, guarded instead of ν: ~2.6 for
/// isotropic metals up to ~20 at the wood extremes, with margin either side.
/// Named constants because the user-facing rejection quotes them.
const MINIMUM_MODULUS_RATIO: f64 = 2.0;
const MAXIMUM_MODULUS_RATIO: f64 = 30.0;

/// `<memberId>/segment/<range>` → the member. Falls back to the whole id.
fn member_of(element_id: &str) -> &str {
    match element_id.find("/segment/") {
        Some(marker) if marker >= 1 => &element_id[..marker],
        _ => element_id,
    }
}

/// Three significant figures: enough to see WHICH side of the band it fell.
fn format_ratio(ratio: f64) -> String {
    if ratio.is_finite() {
        to_precision(ratio, 3)
    } else {
        js_number_to_string(ratio)
    }
}

/// Pa → GPa for display only; the guard itself never leaves pascals.
fn format_modulus_gpa(pascals: f64) -> String {
    if pascals.is_finite() {
        to_precision(pascals / 1e9, 3)
    } else {
        js_number_to_string(pascals)
    }
}

#[derive(Debug, Clone, Copy)]
struct ShearEncoding {
    elastic_modulus_pa: f64,
    nu: f64,
    shear_stiffness_scale: f64,
}

fn encode_moduli(element: &FrameElement) -> PalletResult<ShearEncoding> {
    let e_pa = finite(element.elastic_modulus.value, "elastic modulus")?;
    let g_pa = finite(element.shear_modulus.value, "shear modulus")?;
    // Guard the physically meaningful ratio rather than ν: E/G spans ~2.6
    // (isotropic metals) to ~20 (wood extremes); [2, 30] with margin.
    let ratio = e_pa / g_pa;
    if e_pa <= 0.0
        || g_pa <= 0.0
        || !ratio.is_finite()
        || ratio < MINIMUM_MODULUS_RATIO
        || ratio > MAXIMUM_MODULUS_RATIO
    {
        // Names the MEMBER, the ratio and the band, because this message is what a
        // user sees when an analysis stops: "implausible modulus ratio" alone tells
        // them neither which board is wrong nor by how much. Element IDs are
        // `<memberId>/segment/<range>`, so the member is the part before the marker.
        return unsupported(format!(
            "implausible modulus ratio for {} (element {}): E/G = {} (E = {} GPa, G = {} GPa), outside the supported band {}-{}",
            member_of(&element.element_id),
            element.element_id,
            format_ratio(ratio),
            format_modulus_gpa(e_pa),
            format_modulus_gpa(g_pa),
            js_number_to_string(MINIMUM_MODULUS_RATIO),
            js_number_to_string(MAXIMUM_MODULUS_RATIO),
        ));
    }
    let encoded_shear_modulus_pa = e_pa / (2.0 * (1.0 + ENCODED_POISSON_RATIO));
    Ok(ShearEncoding {
        elastic_modulus_pa: e_pa,
        nu: ENCODED_POISSON_RATIO,
        shear_stiffness_scale: g_pa / encoded_shear_modulus_pa,
    })
}

fn encode_all_moduli(frame: &AnalysisFrame) -> PalletResult<HashMap<String, ShearEncoding>> {
    let mut encodings = HashMap::with_capacity(frame.elements.len());
    for element in &frame.elements {
        encodings.insert(element.element_id.clone(), encode_moduli(element)?);
    }
    Ok(encodings)
}

fn compile_materials(
    frame: &AnalysisFrame,
    ids: &StableIds,
    encodings: &HashMap<String, ShearEncoding>,
) -> PalletResult<HashMap<String, SolverMaterial>> {
    let mut materials = HashMap::with_capacity(frame.elements.len());
    for element in &frame.elements {
        let id = ids.materials.numeric(&format!("material:{}", element.element_id))?;
        let encoding = invariant(encodings.get(&element.element_id), || {
            format!("element {} has no shear encoding", element.element_id)
        })?;
        materials.insert(
            id.to_string(),
            SolverMaterial {
                id,
                e: solver_modulus_value_to_sdk(encoding.elastic_modulus_pa)?,
                nu: encoding.nu,
            },
        );
    }
    Ok(materials)
}

fn compile_sections(
    frame: &AnalysisFrame,
    ids: &StableIds,
    encodings: &HashMap<String, ShearEncoding>,
) -> PalletResult<HashMap<String, SolverSection3D>> {
    let mut sections = HashMap::with_capacity(frame.elements.len());
    for element in &frame.elements {
        let id = ids.sections.numeric(&format!("section:{}", element.element_id))?;
        let encoding = invariant(encodings.get(&element.element_id), || {
            format!("element {} has no shear encoding", element.element_id)
        })?;
        let scale = encoding.shear_stiffness_scale;
        sections.insert(
            id.to_string(),
            SolverSection3D {
                id,
                name: Some(element.element_id.clone()),
                a: element.area_m2,
                iy: element.second_moment_yy_m4,
                iz: element.second_moment_zz_m4,
                // G-carrying terms — scaled so the SDK's reconstructed G reproduces the
                // member's real G·J and G·As products (see ENCODED_POISSON_RATIO).
                // WARNING: post-encoding, sections.asY/asZ/j are STIFFNESS carriers,
                // not geometry. Never read them back as section properties.
                j: element.torsional_constant_m4 * scale,
                cw: None,
                as_y: Some(element.shear_area_y_m2 * scale),
                as_z: Some(element.shear_area_z_m2 * scale),
            },
        );
    }
    Ok(sections)
}

fn assert_no_translational_release(releases: &[bool; 6], element_id: &str) -> PalletResult<()> {
    if releases[0..3].iter().any(|released| *released) {
        return unsupported(format!("translational release on {element_id}"));
    }
    Ok(())
}

fn compile_elements(
    frame: &AnalysisFrame,
    ids: &StableIds,
    triads: &HashMap<String, StabileoLocalTriad>,
) -> PalletResult<HashMap<String, SolverElement3D>> {
    let mut elements = HashMap::with_capacity(frame.elements.len());
    for element in &frame.elements {
        assert_no_translational_release(&element.release_start, &element.element_id)?;
        assert_no_translational_release(&element.release_end, &element.element_id)?;
        let id = ids.elements.numeric(&element.element_id)?;
        let triad = invariant(triads.get(&element.element_id), || {
            format!("element {} has no local triad", element.element_id)
        })?;
        elements.insert(
            id.to_string(),
            SolverElement3D {
                id,
                elem_type: "frame".to_string(),
                node_i: ids.nodes.numeric(&element.start_node_id)?,
                node_j: ids.nodes.numeric(&element.end_node_id)?,
                material_id: ids.materials.numeric(&format!("material:{}", element.element_id))?,
                section_id: ids.sections.numeric(&format!("section:{}", element.element_id))?,
                release_t_start: element.release_start[3],
                release_my_start: element.release_start[4],
                release_mz_start: element.release_start[5],
                release_t_end: element.release_end[3],
                release_my_end: element.release_end[4],
                release_mz_end: element.release_end[5],
                local_yx: Some(finite(triad.y.x, "local y x")?),
                local_yy: Some(finite(triad.y.y, "local y y")?),
                local_yz: Some(finite(triad.y.z, "local y z")?),
                roll_angle: None,
            },
        );
    }
    Ok(elements)
}

fn compile_supports(
    frame: &AnalysisFrame,
    ids: &StableIds,
) -> PalletResult<HashMap<String, SolverSupport3D>> {
    let mut supports = HashMap::new();
    for support in frame.supports.iter().filter(|support| support.active) {
        let [tx, ty, tz, rx, ry, rz] = support.fixed_dofs;
        let translations = support
            .prescribed_translations
            .as_ref()
            .map(|vector| to_stabileo_polar(vector.vec()));
        let rotations =
            support.prescribed_rotations.as_ref().map(|vector| to_stabileo_axial(vector.vec()));
        if let Some(prescribed) = &support.prescribed_translations {
            if prescribed.unit != "m" {
                return unsupported(format!(
                    "prescribed translation for {} must use metres",
                    support.support_id
                ));
            }
        }
        if let Some(prescribed) = &support.prescribed_rotations {
            if prescribed.unit != "rad" {
                return unsupported(format!(
                    "prescribed rotation for {} must use radians",
                    support.support_id
                ));
            }
        }
        let checks: [(Option<f64>, bool, &str); 6] = [
            (translations.map(|vector| vector.x), tx, "TX"),
            (translations.map(|vector| vector.z), ty, "TY"),
            (translations.map(|vector| vector.y), tz, "TZ"),
            (rotations.map(|vector| vector.x), rx, "RX"),
            (rotations.map(|vector| vector.z), ry, "RY"),
            (rotations.map(|vector| vector.y), rz, "RZ"),
        ];
        for (value, fixed, dof) in checks {
            if let Some(value) = value {
                if value.abs() > TOLERANCE && !fixed {
                    return unsupported(format!(
                        "prescribed {} {dof} is not fixed",
                        if dof.starts_with('T') { "translation" } else { "rotation" }
                    ));
                }
            }
        }
        let key = ids.supports.numeric(&support.support_id)?.to_string();
        // ELASTIC AXES, through the same basis swap as everything else — but as a
        // PERMUTATION, not a rotation, and the difference is not cosmetic.
        //
        // `to_stabileo_polar` sends a vector (x, y, z) to (x, -z, y), because the
        // frame's up-axis is the SDK's z and the handedness has to survive. An
        // axial STIFFNESS is not that kind of vector: it is the diagonal of a
        // tensor, one non-negative magnitude per axis, and it changes basis by
        // following its axis to the axis's new name. Signing it would hand the SDK
        // a negative spring on every frame with a lateral stiffness — a structure
        // that pushes harder the further you push it.
        //
        // So the axes travel, exactly as the fixity flags beside them do
        // (`rx: tx, ry: tz, rz: ty`), and the magnitudes do not change:
        // frame X -> kx, frame Z -> ky, frame Y (up) -> kz.
        //
        // UNITS: the SDK's force unit is the KILONEWTON, so a stiffness crosses
        // this boundary in kN/m — `solver_stiffness_value_to_sdk` is that divide by
        // a thousand and nothing else. It matters more than a unit conversion
        // usually does, because the error hides: the floor's 1.0e9 N/m passed
        // through unconverted is a 1.0e12 N/m floor, which is still "rigid" and
        // still returns a solve that looks entirely reasonable.
        //
        // A FIXED axis may not also be elastic. Measured against the real binary:
        // with both `rz: true` and `kz` set, the SDK lets the SPRING govern the
        // displacement and then reports the reaction at that node as ZERO — the
        // load is carried and the bookkeeping never sees it. That is a silent
        // wrong answer, so it is refused here rather than discovered downstream.
        let mut kx = None;
        let mut ky = None;
        let mut kz = None;
        if let Some(stiffness) = &support.elastic_stiffness {
            let axes: [(&mut Option<f64>, f64, bool, &str); 3] = [
                (&mut kx, stiffness.x, tx, "TX"),
                (&mut ky, stiffness.z, tz, "TZ"),
                (&mut kz, stiffness.y, ty, "TY"),
            ];
            for (axis, value, fixed, dof) in axes {
                if finite(value, "support stiffness")? == 0.0 {
                    continue;
                }
                if fixed {
                    return unsupported(format!(
                        "support {} is both fixed and elastic on {dof}",
                        support.support_id
                    ));
                }
                *axis = Some(solver_stiffness_value_to_sdk(value)?);
            }
        }
        supports.insert(
            key,
            SolverSupport3D {
                node_id: ids.nodes.numeric(&support.node_id)?,
                rx: tx,
                ry: tz,
                rz: ty,
                rrx: rx,
                rry: rz,
                rrz: ry,
                kx,
                ky,
                kz,
                krx: None,
                kry: None,
                krz: None,
                dx: translations.map(|vector| vector.x),
                dy: translations.map(|vector| vector.y),
                dz: translations.map(|vector| vector.z),
                drx: rotations.map(|vector| vector.x),
                dry: rotations.map(|vector| vector.y),
                drz: rotations.map(|vector| vector.z),
                rw: None,
                kw: None,
                normal_x: None,
                normal_y: None,
                normal_z: None,
                is_inclined: None,
            },
        );
    }
    Ok(supports)
}

fn compile_constraints(
    constraints: &[ParsedFrameConstraint],
    ids: &StableIds,
) -> PalletResult<Vec<Constraint>> {
    constraints
        .iter()
        .map(|constraint| -> PalletResult<Constraint> {
            ids.constraints.numeric(constraint.constraint_id())?;
            match constraint {
                ParsedFrameConstraint::RigidLink {
                    master_node_id, slave_node_id, dofs, ..
                } => Ok(Constraint::RigidLink(RigidLinkConstraint {
                    master_node: ids.nodes.numeric(master_node_id)?,
                    slave_node: ids.nodes.numeric(slave_node_id)?,
                    dofs: dofs.iter().map(|dof| sdk_dof(dof)).collect::<PalletResult<Vec<usize>>>()?,
                })),
                ParsedFrameConstraint::EqualDof {
                    master_node_id, slave_node_id, dofs, ..
                } => Ok(Constraint::EqualDOF(EqualDOFConstraint {
                    master_node: ids.nodes.numeric(master_node_id)?,
                    slave_node: ids.nodes.numeric(slave_node_id)?,
                    dofs: dofs.iter().map(|dof| sdk_dof(dof)).collect::<PalletResult<Vec<usize>>>()?,
                })),
                ParsedFrameConstraint::LinearMpc { terms, .. } => {
                    Ok(Constraint::LinearMPC(LinearMPCConstraint {
                        terms: terms
                            .iter()
                            .map(|term| -> PalletResult<MPCTerm> {
                                Ok(MPCTerm {
                                    node_id: ids.nodes.numeric(&term.node_id)?,
                                    dof: sdk_dof(&term.dof)?,
                                    coefficient: term.coefficient,
                                })
                            })
                            .collect::<PalletResult<Vec<MPCTerm>>>()?,
                    }))
                }
                ParsedFrameConstraint::EccentricConnection {
                    master_node_id,
                    slave_node_id,
                    polar_offset,
                    releases,
                    ..
                } => {
                    let offset = to_stabileo_polar(polar_offset.vec());
                    Ok(Constraint::EccentricConnection(EccentricConnectionConstraint {
                        master_node: ids.nodes.numeric(master_node_id)?,
                        slave_node: ids.nodes.numeric(slave_node_id)?,
                        offset_x: offset.x,
                        offset_y: offset.y,
                        offset_z: offset.z,
                        releases: vec![
                            releases.tx,
                            releases.tz,
                            releases.ty,
                            releases.rx,
                            releases.rz,
                            releases.ry,
                        ],
                    }))
                }
            }
        })
        .collect()
}

fn compile_connectors(
    frame: &AnalysisFrame,
    ids: &StableIds,
) -> PalletResult<HashMap<String, ConnectorElement>> {
    let mut connectors = HashMap::with_capacity(frame.connectors.len());
    for connector in &frame.connectors {
        let id = ids.connectors.numeric(&connector.connector_id)?;
        // A CONNECTOR'S STIFFNESSES ARE LOCAL — axial along its own axis, the
        // other two across it — so unlike a support's they are not a global
        // vector and there is no basis swap to apply here. The convention that
        // DOES matter is which transverse direction is which, and it is measured
        // rather than assumed: for a connector whose axis is the frame's up-axis
        // (every pallet joint — a deckboard above a stringer), the SDK acts
        // `kShear` along the axis `normalize.rs`'s own `connector_triad` calls
        // local y and `kShearZ` along the one it calls local z, so this mapping
        // is the agreeing one. Pinned in qualification/elastic-support.test.ts;
        // NOT established for any other connector orientation.
        //
        // Converted as STIFFNESSES (N/m -> kN/m), which is the same arithmetic a
        // force gets and a different statement about what the number is.
        //
        // An absent stiffness is an absent key on the TS wire, which the
        // kernel's `#[serde(default)]` reads back as zero — so absent is zero
        // here too, written straight into the field.
        let stiffness = |quantity: &Option<Quantity>| -> PalletResult<f64> {
            match quantity {
                Some(quantity) => solver_stiffness_value_to_sdk(quantity.value),
                None => Ok(0.0),
            }
        };
        let moment_stiffness = |quantity: &Option<Quantity>| -> PalletResult<f64> {
            match quantity {
                Some(quantity) => solver_moment_value_to_sdk(quantity.value),
                None => Ok(0.0),
            }
        };
        connectors.insert(
            id.to_string(),
            ConnectorElement {
                id,
                node_i: ids.nodes.numeric(&connector.node_i)?,
                node_j: ids.nodes.numeric(&connector.node_j)?,
                k_axial: stiffness(&connector.axial_stiffness)?,
                k_shear: stiffness(&connector.shear_y_stiffness)?,
                k_shear_z: stiffness(&connector.shear_z_stiffness)?,
                k_moment: moment_stiffness(&connector.torsion_stiffness)?,
                k_bend_y: moment_stiffness(&connector.bend_y_stiffness)?,
                k_bend_z: moment_stiffness(&connector.bend_z_stiffness)?,
            },
        );
    }
    Ok(connectors)
}

struct ElementData<'a> {
    #[allow(dead_code)]
    element: &'a FrameElement,
    start: Vec3,
    #[allow(dead_code)]
    end: Vec3,
    delta: Vec3,
    length: f64,
    /// `calculateResultant` calls `elementData` with an EMPTY triad map and
    /// only reads `start`, `delta` and `length`; the TS `!` on the lookup is
    /// undefined there and never dereferenced, so the triad is optional here.
    triad: Option<StabileoLocalTriad>,
}

fn element_data<'a>(
    frame: &'a AnalysisFrame,
    element_id: &str,
    triads: &HashMap<String, StabileoLocalTriad>,
) -> PalletResult<ElementData<'a>> {
    let element = invariant(
        frame.elements.iter().find(|candidate| candidate.element_id == element_id),
        || format!("load names an unknown element {element_id}"),
    )?;
    let start = invariant(
        frame.nodes.iter().find(|node| node.node_id == element.start_node_id),
        || format!("element {} names an unknown start node {}", element.element_id, element.start_node_id),
    )?
    .position
    .vec();
    let end = invariant(
        frame.nodes.iter().find(|node| node.node_id == element.end_node_id),
        || format!("element {} names an unknown end node {}", element.element_id, element.end_node_id),
    )?
    .position
    .vec();
    let delta = subtract(end, start);
    let length = norm(delta);
    Ok(ElementData { element, start, end, delta, length, triad: triads.get(element_id).copied() })
}

fn project_local(vector: Vec3, triad: &StabileoLocalTriad) -> Vec3 {
    Vec3 { x: dot(vector, triad.x), y: dot(vector, triad.y), z: dot(vector, triad.z) }
}

fn compile_loads(
    frame: &AnalysisFrame,
    ids: &StableIds,
    triads: &HashMap<String, StabileoLocalTriad>,
) -> PalletResult<Vec<SolverLoad3D>> {
    frame
        .loads
        .iter()
        .map(|load| -> PalletResult<SolverLoad3D> {
            match load {
                FrameLoad::NodalForce { load_id, node_id, force, .. } => {
                    if force.unit != "N" {
                        return unsupported(format!("nodal force {load_id} must use newtons"));
                    }
                    let force = to_stabileo_polar(force.vec());
                    Ok(SolverLoad3D::Nodal(SolverNodalLoad3D {
                        node_id: ids.nodes.numeric(node_id)?,
                        fx: solver_force_value_to_sdk(force.x)?,
                        fy: solver_force_value_to_sdk(force.y)?,
                        fz: solver_force_value_to_sdk(force.z)?,
                        mx: 0.0,
                        my: 0.0,
                        mz: 0.0,
                        bw: None,
                    }))
                }
                FrameLoad::NodalMoment { load_id, node_id, moment, .. } => {
                    if moment.unit != "N_m" {
                        return unsupported(format!(
                            "nodal moment {load_id} must use newton-metres"
                        ));
                    }
                    let moment = to_stabileo_axial(moment.vec());
                    Ok(SolverLoad3D::Nodal(SolverNodalLoad3D {
                        node_id: ids.nodes.numeric(node_id)?,
                        fx: 0.0,
                        fy: 0.0,
                        fz: 0.0,
                        mx: solver_moment_value_to_sdk(moment.x)?,
                        my: solver_moment_value_to_sdk(moment.y)?,
                        mz: solver_moment_value_to_sdk(moment.z)?,
                        bw: None,
                    }))
                }
                FrameLoad::ElementPointForce {
                    load_id,
                    element_id,
                    normalized_position,
                    global_application_point,
                    force,
                    moment,
                    ..
                } => {
                    let data = element_data(frame, element_id, triads)?;
                    let expected = add(data.start, scale(data.delta, *normalized_position));
                    if norm(subtract(global_application_point.vec(), expected))
                        > TOLERANCE * 1.0_f64.max(data.length)
                    {
                        return unsupported(format!(
                            "application point for {load_id} does not lie at normalizedPosition"
                        ));
                    }
                    if norm(moment.vec()) > TOLERANCE {
                        return unsupported(format!(
                            "point moment on {load_id} is not representable by Stabileo {STABILEO_SDK_VERSION}"
                        ));
                    }
                    let triad = invariant(data.triad.as_ref(), || {
                        format!("element {element_id} has no local triad")
                    })?;
                    let local = project_local(to_stabileo_polar(force.vec()), triad);
                    if local.x.abs() > TOLERANCE {
                        return unsupported(format!(
                            "axial point force on {load_id} is not representable by Stabileo {STABILEO_SDK_VERSION}"
                        ));
                    }
                    Ok(SolverLoad3D::PointOnElement(SolverPointLoad3D {
                        element_id: ids.elements.numeric(element_id)?,
                        a: normalized_position * data.length,
                        py: solver_force_value_to_sdk(local.y)?,
                        pz: solver_force_value_to_sdk(local.z)?,
                    }))
                }
                FrameLoad::ElementDistributedForce {
                    load_id,
                    element_id,
                    start_position,
                    end_position,
                    force_per_metre,
                    ..
                } => {
                    let data = element_data(frame, element_id, triads)?;
                    let triad = invariant(data.triad.as_ref(), || {
                        format!("element {element_id} has no local triad")
                    })?;
                    let local = project_local(to_stabileo_polar(force_per_metre.vec()), triad);
                    if local.x.abs() > TOLERANCE {
                        return unsupported(format!(
                            "axial distributed force on {load_id} is not representable by Stabileo {STABILEO_SDK_VERSION}"
                        ));
                    }
                    Ok(SolverLoad3D::Distributed(SolverDistributedLoad3D {
                        element_id: ids.elements.numeric(element_id)?,
                        q_yi: solver_force_value_to_sdk(local.y)?,
                        q_yj: solver_force_value_to_sdk(local.y)?,
                        q_zi: solver_force_value_to_sdk(local.z)?,
                        q_zj: solver_force_value_to_sdk(local.z)?,
                        a: Some(start_position * data.length),
                        b: Some(end_position * data.length),
                    }))
                }
            }
        })
        .collect()
}

fn calculate_resultant(frame: &AnalysisFrame) -> PalletResult<Resultant> {
    let mut force = Vec3::ZERO;
    let mut moment = Vec3::ZERO;
    let node_by_id: HashMap<&str, Vec3> =
        frame.nodes.iter().map(|node| (node.node_id.as_str(), node.position.vec())).collect();
    for load in &frame.loads {
        let mut load_force = Vec3::ZERO;
        let mut load_moment = Vec3::ZERO;
        let mut application_point = Vec3::ZERO;
        match load {
            FrameLoad::NodalForce { node_id, force: nodal_force, .. } => {
                load_force = nodal_force.vec();
                application_point = *invariant(node_by_id.get(node_id.as_str()), || {
                    format!("load names an unknown node {node_id}")
                })?;
            }
            FrameLoad::NodalMoment { moment: nodal_moment, .. } => {
                load_moment = nodal_moment.vec();
            }
            FrameLoad::ElementPointForce {
                force: point_force, moment: point_moment, global_application_point, ..
            } => {
                load_force = point_force.vec();
                load_moment = point_moment.vec();
                application_point = global_application_point.vec();
            }
            FrameLoad::ElementDistributedForce {
                element_id, start_position, end_position, force_per_metre, ..
            } => {
                let data = element_data(frame, element_id, &HashMap::new())?;
                let loaded_length = (end_position - start_position) * data.length;
                load_force = scale(force_per_metre.vec(), loaded_length);
                application_point =
                    add(data.start, scale(data.delta, (start_position + end_position) / 2.0));
            }
        }
        force = add(force, load_force);
        moment = add(moment, add(load_moment, cross(application_point, load_force)));
    }
    let newton = |value: f64| -> PalletResult<Quantity> {
        Ok(Quantity { unit: "N".to_string(), value: finite(value, "resultant force")? })
    };
    let newton_metre = |value: f64| -> PalletResult<Quantity> {
        Ok(Quantity { unit: "N_m".to_string(), value: finite(value, "resultant moment")? })
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

/// TS `compileStaticActiveState`, minus the zod parse of the request (the
/// caller hands typed structs) and minus the capability gate (see
/// `SUPPORTED_CAPABILITIES`).
pub fn compile_static_active_state(
    frame: &AnalysisFrame,
    request_id: &str,
    active_state_id: &str,
    active_state_hash: &str,
) -> PalletResult<CompiledStabileoModel> {
    let parsed_constraints = parse_frame_constraints(&frame.constraints)?;
    let ids = ids_for(frame, &parsed_constraints)?;
    let element_triads = build_triads(frame)?;
    let constraints = compile_constraints(&parsed_constraints, &ids)?;
    let shear_encodings = encode_all_moduli(frame)?;
    let input = SolverInput3D {
        nodes: compile_nodes(frame, &ids)?,
        materials: compile_materials(frame, &ids, &shear_encodings)?,
        sections: compile_sections(frame, &ids, &shear_encodings)?,
        elements: compile_elements(frame, &ids, &element_triads)?,
        supports: compile_supports(frame, &ids)?,
        loads: compile_loads(frame, &ids, &element_triads)?,
        // `SolverInput3D` carries the constraint list too, and it reaches the
        // same solver — so it takes the same list.
        constraints: constraints.clone(),
        left_hand: Some(false),
        plates: HashMap::new(),
        quads: HashMap::new(),
        quad9s: HashMap::new(),
        solid_shells: HashMap::new(),
        curved_shells: HashMap::new(),
        curved_beams: Vec::new(),
        connectors: compile_connectors(frame, &ids)?,
    };
    Ok(CompiledStabileoModel {
        frame: frame.clone(),
        request_id: request_id.to_string(),
        input,
        constraints,
        parsed_constraints,
        ids,
        element_triads,
        applied_resultant: calculate_resultant(frame)?,
        active_state_id: active_state_id.to_string(),
        active_state_hash: active_state_hash.to_string(),
    })
}
