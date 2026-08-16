//! Serde mirrors of the application contracts the coupled solve consumes and
//! produces. These are TRANSPORT shapes: every struct that crosses JSON keeps
//! unknown fields in a flattened `extra` map so state the logic does not
//! touch round-trips intact, and tagged vectors keep their `kind`/`unit` tags
//! as data. Validation lives with the application's zod schemas on the far
//! side of the boundary; this side trusts its caller and enforces only what
//! the arithmetic needs.

use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};

use crate::types::Vec3;

pub type Extra = Map<String, Value>;

/// `{ unit, value }` quantity wrapper (Newton, Metre, Ratio, …).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Quantity {
    pub unit: String,
    pub value: f64,
}

/// Tagged 3-vector: `{ kind: POINT|POLAR_VECTOR|AXIAL_VECTOR, unit, x, y, z }`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Tagged3 {
    pub kind: String,
    pub unit: String,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Tagged3 {
    pub fn vec(&self) -> Vec3 {
        Vec3 { x: self.x, y: self.y, z: self.z }
    }

    pub fn point_m(v: Vec3) -> Tagged3 {
        Tagged3 { kind: "POINT".into(), unit: "m".into(), x: v.x, y: v.y, z: v.z }
    }

    pub fn polar(unit: &str, v: Vec3) -> Tagged3 {
        Tagged3 { kind: "POLAR_VECTOR".into(), unit: unit.into(), x: v.x, y: v.y, z: v.z }
    }

    pub fn axial(unit: &str, v: Vec3) -> Tagged3 {
        Tagged3 { kind: "AXIAL_VECTOR".into(), unit: unit.into(), x: v.x, y: v.y, z: v.z }
    }
}

/// Planar `{ x, z }` numeric pair (tangential floor-plane quantities).
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PlainXZ {
    pub x: f64,
    pub z: f64,
}

/// Plain numeric `{ x, y, z }` (service-event accelerations).
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PlainXYZ {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

// ---------------------------------------------------------------------------
// Frame
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FrameNode {
    pub node_id: String,
    pub position: Tagged3,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FrameElement {
    pub element_id: String,
    pub start_node_id: String,
    pub end_node_id: String,
    pub area_m2: f64,
    #[serde(rename = "shearAreaYM2")]
    pub shear_area_y_m2: f64,
    #[serde(rename = "shearAreaZM2")]
    pub shear_area_z_m2: f64,
    pub torsional_constant_m4: f64,
    pub second_moment_yy_m4: f64,
    pub second_moment_zz_m4: f64,
    pub elastic_modulus: Quantity,
    pub shear_modulus: Quantity,
    #[serde(rename = "localYAxis")]
    pub local_y_axis: Tagged3,
    pub release_start: [bool; 6],
    pub release_end: [bool; 6],
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FrameConnector {
    pub connector_id: String,
    #[serde(rename = "nodeI")]
    pub node_i: String,
    #[serde(rename = "nodeJ")]
    pub node_j: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub axial_stiffness: Option<Quantity>,
    #[serde(rename = "shearYStiffness", skip_serializing_if = "Option::is_none")]
    pub shear_y_stiffness: Option<Quantity>,
    #[serde(rename = "shearZStiffness", skip_serializing_if = "Option::is_none")]
    pub shear_z_stiffness: Option<Quantity>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub torsion_stiffness: Option<Quantity>,
    #[serde(rename = "bendYStiffness", skip_serializing_if = "Option::is_none")]
    pub bend_y_stiffness: Option<Quantity>,
    #[serde(rename = "bendZStiffness", skip_serializing_if = "Option::is_none")]
    pub bend_z_stiffness: Option<Quantity>,
}

/// Constraints are consumed structurally by the kernel bridge but never
/// mutated by the loop — carried as raw values.
pub type FrameConstraint = Value;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FrameSupport {
    pub support_id: String,
    pub node_id: String,
    pub active: bool,
    pub fixed_dofs: [bool; 6],
    pub prescribed_translations: Option<Tagged3>,
    pub prescribed_rotations: Option<Tagged3>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub elastic_stiffness: Option<Tagged3>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FrameLoadApplication {
    pub source_kind: String,
    pub source_id: String,
    pub parent_member_id: Option<String>,
    pub contact_id: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "kind")]
pub enum FrameLoad {
    #[serde(rename = "NODAL_FORCE", rename_all = "camelCase")]
    NodalForce {
        load_id: String,
        node_id: String,
        force: Tagged3,
        #[serde(skip_serializing_if = "Option::is_none")]
        application: Option<FrameLoadApplication>,
    },
    #[serde(rename = "NODAL_MOMENT", rename_all = "camelCase")]
    NodalMoment {
        load_id: String,
        node_id: String,
        moment: Tagged3,
        #[serde(skip_serializing_if = "Option::is_none")]
        application: Option<FrameLoadApplication>,
    },
    #[serde(rename = "ELEMENT_POINT_FORCE", rename_all = "camelCase")]
    ElementPointForce {
        load_id: String,
        element_id: String,
        normalized_position: f64,
        global_application_point: Tagged3,
        force: Tagged3,
        moment: Tagged3,
        application: FrameLoadApplication,
    },
    #[serde(rename = "ELEMENT_DISTRIBUTED_FORCE", rename_all = "camelCase")]
    ElementDistributedForce {
        load_id: String,
        element_id: String,
        start_position: f64,
        end_position: f64,
        force_per_metre: Tagged3,
        application: FrameLoadApplication,
    },
}

impl FrameLoad {
    pub fn load_id(&self) -> &str {
        match self {
            FrameLoad::NodalForce { load_id, .. }
            | FrameLoad::NodalMoment { load_id, .. }
            | FrameLoad::ElementPointForce { load_id, .. }
            | FrameLoad::ElementDistributedForce { load_id, .. } => load_id,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct AnalysisFrame {
    pub schema_version: String,
    pub frame_id: String,
    pub coordinate_basis: String,
    pub nodes: Vec<FrameNode>,
    pub elements: Vec<FrameElement>,
    pub supports: Vec<FrameSupport>,
    pub loads: Vec<FrameLoad>,
    pub connectors: Vec<FrameConnector>,
    pub constraints: Vec<FrameConstraint>,
    /// Application identity; carried, never recomputed inside the loop.
    pub frame_hash: String,
}

/// `{ force: {x: Quantity, ...}, moment: {x: Quantity, ...} }`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResultantAxes {
    pub x: Quantity,
    pub y: Quantity,
    pub z: Quantity,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Resultant {
    pub force: ResultantAxes,
    pub moment: ResultantAxes,
}

// ---------------------------------------------------------------------------
// Kernel result (app-normalized shape)
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct KernelNodeResponse {
    pub node_id: String,
    pub translation: Tagged3,
    pub rotation: Tagged3,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct KernelReaction {
    pub support_id: String,
    pub force: Tagged3,
    pub moment: Tagged3,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct KernelElementEndForces {
    pub element_id: String,
    pub start_force: Tagged3,
    pub start_moment: Tagged3,
    pub end_force: Tagged3,
    pub end_moment: Tagged3,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct KernelDiagnostic {
    pub code: String,
    pub severity: String,
    pub entity_id: Option<String>,
    pub message: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct KernelResult {
    pub schema_version: String,
    pub request_id: String,
    pub active_state_id: String,
    pub active_state_hash: String,
    pub node_responses: Vec<KernelNodeResponse>,
    pub reactions: Vec<KernelReaction>,
    pub element_end_forces: Vec<KernelElementEndForces>,
    /// Connector/constraint force detail: consumed downstream by recovery,
    /// carried opaque through the loop.
    pub connector_responses: Vec<Value>,
    pub constraint_forces: Vec<Value>,
    pub applied_resultant: Resultant,
    pub reaction_resultant: Resultant,
    pub force_residual: Tagged3,
    pub moment_residual: Tagged3,
    pub diagnostics: Vec<KernelDiagnostic>,
    pub result_hash: String,
}

// ---------------------------------------------------------------------------
// Support state
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SupportContactActiveStateEntry {
    pub support_id: String,
    pub contact_id: String,
    pub normal_state: String,
    #[serde(rename = "verticalCompressionM")]
    pub vertical_compression_m: f64,
    #[serde(rename = "verticalReactionN")]
    pub vertical_reaction_n: f64,
    pub tangential_state: String,
    #[serde(rename = "tangentialDisplacementM")]
    pub tangential_displacement_m: PlainXZ,
    #[serde(rename = "tangentialReactionN")]
    pub tangential_reaction_n: PlainXZ,
    pub friction_utilization: Option<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct BearingContactMechanics {
    pub vertical_stiffness: Quantity,
    pub bearing_capacity: Quantity,
    pub friction_coefficient: Quantity,
    pub settlement: Quantity,
    #[serde(flatten)]
    pub extra: Extra,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PalletBearingContact {
    pub contact_id: String,
    pub support_id: String,
    pub node_id: String,
    pub mechanics: BearingContactMechanics,
    #[serde(flatten)]
    pub extra: Extra,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct CompiledPalletSupportState {
    pub schema_version: String,
    pub condition_id: String,
    pub frame_supports: Vec<FrameSupport>,
    pub bearing_contacts: Vec<PalletBearingContact>,
    pub active_state: Vec<SupportContactActiveStateEntry>,
    /// Application identity of the active set; stamped app-side. The loop
    /// carries the incoming value and the app re-derives it on the terminal
    /// state it gets back.
    #[serde(rename = "activeStateSha256")]
    pub active_state_sha256: String,
    #[serde(flatten)]
    pub extra: Extra,
}

// ---------------------------------------------------------------------------
// Contact patches, projection, top response
// ---------------------------------------------------------------------------

/// A pallet-top contact patch. The loop rewrites `center`, `force`,
/// `freeMoment`, `orientation`, `boundary`, and `samples`; every other field
/// (kind-specific dimensions, ids, stiffness) passes through `extra`. The
/// `kind` tag is data here so all five variants share one struct.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PalletTopContactPatch {
    pub kind: String,
    pub contact_id: String,
    pub center: Tagged3,
    pub force: Tagged3,
    pub free_moment: Tagged3,
    #[serde(rename = "normalStiffnessNPerM")]
    pub normal_stiffness_n_per_m: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub orientation: Option<Quantity>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub boundary: Option<Vec<Tagged3>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub samples: Option<Vec<ContactPressureSample>>,
    #[serde(rename = "halfSizeX", skip_serializing_if = "Option::is_none")]
    pub half_size_x: Option<Quantity>,
    #[serde(rename = "halfSizeZ", skip_serializing_if = "Option::is_none")]
    pub half_size_z: Option<Quantity>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub half_length: Option<Quantity>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub half_width: Option<Quantity>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub inner_radius: Option<Quantity>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub outer_radius: Option<Quantity>,
    #[serde(flatten)]
    pub extra: Extra,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ContactPressureSample {
    pub point: Tagged3,
    pub normalized_weight: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ContactFaceSystem {
    pub nodes: Vec<FrameNode>,
    pub constraints: Vec<FrameConstraint>,
    pub connectors: Vec<FrameConnector>,
    pub supports: Vec<FrameSupport>,
    pub loads: Vec<FrameLoad>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ContactResponsePoint {
    pub response_point_id: String,
    pub element_id: String,
    pub element_natural_coordinate: f64,
    pub global_point: Tagged3,
    pub rigid_offset_from_element_axis: Tagged3,
    pub normalized_contact_weight: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ContactMapEntry {
    pub contact_id: String,
    pub response_points: Vec<ContactResponsePoint>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ResultantConservationAudit {
    pub input_resultant: Resultant,
    pub projected_resultant: Resultant,
    #[serde(rename = "forceResidualNormN")]
    pub force_residual_norm_n: f64,
    #[serde(rename = "momentResidualNormNm")]
    pub moment_residual_norm_nm: f64,
    #[serde(rename = "resultantLocationResidualM")]
    pub resultant_location_residual_m: f64,
    pub accepted: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PalletContactProjectionResult {
    pub schema_version: String,
    pub loads: Vec<FrameLoad>,
    pub face_system: ContactFaceSystem,
    pub contact_map: Vec<ContactMapEntry>,
    pub audit: ResultantConservationAudit,
    #[serde(flatten)]
    pub extra: Extra,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PalletTopContactResponse {
    pub contact_id: String,
    pub translation: Tagged3,
    pub rotation: Tagged3,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PalletTopResponse {
    pub schema_version: String,
    pub frame_hash: String,
    pub kernel_result_hash: String,
    pub contacts: Vec<PalletTopContactResponse>,
    #[serde(rename = "responseSha256")]
    pub response_sha256: String,
}

// ---------------------------------------------------------------------------
// Unit load
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct UnitLoadEvent {
    pub load_event_id: String,
    pub kind: String,
    #[serde(rename = "accelerationMPerS2")]
    pub acceleration_m_per_s2: PlainXYZ,
    #[serde(rename = "angularAccelerationRadPerS2")]
    pub angular_acceleration_rad_per_s2: PlainXYZ,
    #[serde(flatten)]
    pub extra: Extra,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct UnitLoadInterface {
    pub schema_version: String,
    pub interface_id: String,
    pub lower_package_instance_id: Option<String>,
    pub upper_package_instance_id: String,
    pub contact_id: String,
    pub load_share_ratio: f64,
    #[serde(rename = "normalStiffnessNPerM")]
    pub normal_stiffness_n_per_m: f64,
    pub friction_coefficient: f64,
    #[serde(rename = "shearCapacityN")]
    pub shear_capacity_n: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct UnitLoadInterfaceState {
    pub interface_id: String,
    pub normal_state: String,
    pub tangent_state: String,
    pub slip_direction_x: f64,
    pub slip_direction_z: f64,
    #[serde(rename = "normalForceN")]
    pub normal_force_n: f64,
    #[serde(rename = "tangentForceXN")]
    pub tangent_force_x_n: f64,
    #[serde(rename = "tangentForceZN")]
    pub tangent_force_z_n: f64,
    #[serde(rename = "complementarityResidualN")]
    pub complementarity_residual_n: f64,
    #[serde(rename = "dissipatedEnergyJ")]
    pub dissipated_energy_j: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct UnitLoadForceApplication {
    pub application_id: String,
    pub source_kind: String,
    pub source_id: String,
    pub point: Tagged3,
    pub force: Tagged3,
    pub moment: Tagged3,
}

/// Criterion results: three structurally distinct variants in the app's
/// union. Untagged deserialization tries them in declaration order — the
/// finite-capacity shape first, exactly as the zod union does.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum UnitLoadCriterionResult {
    #[serde(rename_all = "camelCase")]
    Finite {
        criterion_id: String,
        kind: String,
        classification: String,
        demand: f64,
        capacity: f64,
        utilization: f64,
        governing_entity_id: String,
    },
    #[serde(rename_all = "camelCase")]
    ZeroCapacity {
        criterion_id: String,
        kind: String,
        classification: String,
        demand: f64,
        capacity: f64,
        utilization: Option<f64>,
        reason_code: String,
        governing_entity_id: String,
    },
}

impl UnitLoadCriterionResult {
    pub fn kind(&self) -> &str {
        match self {
            UnitLoadCriterionResult::Finite { kind, .. }
            | UnitLoadCriterionResult::ZeroCapacity { kind, .. } => kind,
        }
    }

    pub fn criterion_id(&self) -> &str {
        match self {
            UnitLoadCriterionResult::Finite { criterion_id, .. }
            | UnitLoadCriterionResult::ZeroCapacity { criterion_id, .. } => criterion_id,
        }
    }

    pub fn governing_entity_id(&self) -> &str {
        match self {
            UnitLoadCriterionResult::Finite { governing_entity_id, .. }
            | UnitLoadCriterionResult::ZeroCapacity { governing_entity_id, .. } => governing_entity_id,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct RememberedBaseLoadShare {
    pub interface_id: String,
    pub load_share_ratio: f64,
    #[serde(rename = "deckComplianceMPerN")]
    pub deck_compliance_m_per_n: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct UnitLoadActiveState {
    pub schema_version: String,
    pub service_event: UnitLoadEvent,
    pub pallet_response: Option<PalletTopResponse>,
    pub interfaces: Vec<UnitLoadInterface>,
    pub interface_states: Vec<UnitLoadInterfaceState>,
    pub pallet_contacts: Vec<PalletTopContactPatch>,
    pub force_applications: Vec<UnitLoadForceApplication>,
    pub criteria: Vec<UnitLoadCriterionResult>,
    #[serde(default)]
    pub previous_base_load_shares: Option<Vec<RememberedBaseLoadShare>>,
    /// Application identity; carried and re-derived app-side.
    #[serde(rename = "activeStateSha256")]
    pub active_state_sha256: String,
    /// bodies, sampleDraws, packageType, scenarioType, manifest hashes — the
    /// loop never reads them; byte-faithful passthrough.
    #[serde(flatten)]
    pub extra: Extra,
}

// ---------------------------------------------------------------------------
// Numerical profile
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct NumericalAcceptanceProfile {
    pub schema_version: String,
    pub profile_id: String,
    #[serde(rename = "profileSha256")]
    pub profile_sha256: String,
    #[serde(rename = "geometryToleranceM")]
    pub geometry_tolerance_m: f64,
    #[serde(rename = "forceToleranceN")]
    pub force_tolerance_n: f64,
    #[serde(rename = "momentToleranceNm")]
    pub moment_tolerance_nm: f64,
    #[serde(rename = "lengthToleranceM")]
    pub length_tolerance_m: f64,
    #[serde(rename = "complementarityToleranceN")]
    pub complementarity_tolerance_n: f64,
    pub coupled_iteration_limit: u32,
    #[serde(rename = "coupledTranslationToleranceM")]
    pub coupled_translation_tolerance_m: f64,
    #[serde(rename = "coupledRotationToleranceRad")]
    pub coupled_rotation_tolerance_rad: f64,
    pub coupled_load_share_tolerance: f64,
}
