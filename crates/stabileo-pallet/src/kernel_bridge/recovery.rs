//! Literal port of `packages/analysis/stabileo/src/recovery.ts`: the seven
//! numbers per station, recovered from the kernel's own beam-station
//! extraction and its diagram evaluator, in the application's units.

use dedaliano_engine::postprocess::beam_stations::{
    extract_beam_stations_3d, BeamMemberInfo, BeamStationInput3D, LabeledResults3D,
};
use dedaliano_engine::postprocess::diagrams_3d::evaluate_diagram_3d_at;
use dedaliano_engine::types::AnalysisResults3D;
use serde::{Deserialize, Serialize};

use crate::kernel_bridge::compile::CompiledStabileoModel;
use crate::kernel_bridge::id_map::bytewise_utf8_compare;
use crate::kernel_bridge::units::{sdk_force_value_to_solver, sdk_moment_value_to_solver};
use crate::types::{PalletError, PalletResult};

/// TS `FrameResponseRecoveryRequest.stationCounts[]`.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FrameResponseStationCount {
    pub element_id: String,
    pub count: usize,
}

/// One recovered station. The field names are the app's
/// `FrameResponseRecoveryResultSchema` names; serde's camelCase reproduces
/// each one exactly (`axial_force_n` -> `axialForceN`, `torsion_nm` ->
/// `torsionNm`).
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FrameResponseStation {
    pub normalized_position: f64,
    pub axial_force_n: f64,
    pub shear_y_n: f64,
    pub shear_z_n: f64,
    pub torsion_nm: f64,
    pub bending_y_nm: f64,
    pub bending_z_nm: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FrameResponseRecoveryElement {
    pub element_id: String,
    pub stations: Vec<FrameResponseStation>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FrameResponseRecovery {
    pub schema_version: String,
    pub request_id: String,
    pub elements: Vec<FrameResponseRecoveryElement>,
    /// PORTING.md rule 7: the TS stamps `sha256CanonicalSync(payload)`; the
    /// port carries the field and computes no hash.
    pub response_hash: String,
}

fn finite(value: f64, label: &str) -> PalletResult<f64> {
    if !value.is_finite() {
        return Err(PalletError::sentence(format!(
            "CORRUPT_OUTPUT: non-finite recovered {label}"
        )));
    }
    Ok(if value == 0.0 { 0.0 } else { value })
}

/// TS `recoverFrameResponse`.
///
/// PORT-QUESTION: the TS opens with two identity guards
/// (`RECOVERY_REQUEST_IDENTITY_MISMATCH`, `RECOVERY_RESULT_IDENTITY_MISMATCH`)
/// and the kernel wrapper adds a third (`RECOVERY_RESULT_NOT_CACHED`), because
/// the TS bridges two separate WASM calls through a result-hash cache. Here
/// the caller hands the compiled model and the raw results it solved, so there
/// are no two identities to disagree — the guards have no input and are not
/// ported. Everything downstream of them is.
pub fn recover_frame_response(
    compiled: &CompiledStabileoModel,
    raw: &AnalysisResults3D,
    station_counts: &[FrameResponseStationCount],
) -> PalletResult<FrameResponseRecovery> {
    let mut seen: Vec<&str> = Vec::with_capacity(station_counts.len());
    let mut elements = Vec::with_capacity(station_counts.len());
    for FrameResponseStationCount { element_id, count } in station_counts {
        if seen.contains(&element_id.as_str()) {
            return Err(PalletError::sentence(format!(
                "RECOVERY_DUPLICATE_ELEMENT: {element_id}"
            )));
        }
        seen.push(element_id);
        let numeric_id = compiled.ids.elements.numeric(element_id)?;
        let Some(raw_forces) =
            raw.element_forces.iter().find(|item| item.element_id == numeric_id)
        else {
            return Err(PalletError::sentence(format!(
                "RECOVERY_ELEMENT_RESULT_MISSING: {element_id}"
            )));
        };
        let Some(input_element) = compiled.input.elements.get(&numeric_id.to_string()) else {
            // TS `compiled.input.elements[String(numericId)]!` — the compiled
            // element map is keyed by the same numeric ids.
            return Err(PalletError::sentence(format!(
                "RECOVERY_ELEMENT_RESULT_MISSING: {element_id}"
            )));
        };
        let station_result = extract_beam_stations_3d(&BeamStationInput3D {
            members: vec![BeamMemberInfo {
                element_id: numeric_id,
                section_id: input_element.section_id,
                material_id: input_element.material_id,
                length: raw_forces.length,
                label: Some(element_id.clone()),
            }],
            combinations: vec![LabeledResults3D {
                combo_id: 1,
                combo_name: Some("ACTIVE_STATE".to_string()),
                results: raw.clone(),
            }],
            num_stations: Some(*count),
        });
        let positions: Vec<f64> = station_result
            .stations
            .iter()
            .map(|station| finite(station.t, "station position"))
            .collect::<PalletResult<Vec<f64>>>()?;
        if positions.len() != *count
            || positions.iter().enumerate().any(|(index, position)| {
                *position < 0.0
                    || *position > 1.0
                    || (index > 0 && *position <= positions[index - 1])
            })
        {
            return Err(PalletError::sentence(format!(
                "CORRUPT_OUTPUT: invalid station set for {element_id}"
            )));
        }
        let value = |kind: &str, position: f64| -> PalletResult<f64> {
            let raw_value = finite(evaluate_diagram_3d_at(raw_forces, kind, position), kind)?;
            if kind == "axial" || kind == "shearY" || kind == "shearZ" {
                sdk_force_value_to_solver(raw_value)
            } else {
                sdk_moment_value_to_solver(raw_value)
            }
        };
        let mut stations = Vec::with_capacity(positions.len());
        for position in positions {
            stations.push(FrameResponseStation {
                normalized_position: position,
                axial_force_n: value("axial", position)?,
                shear_y_n: value("shearY", position)?,
                shear_z_n: value("shearZ", position)?,
                torsion_nm: value("torsion", position)?,
                bending_y_nm: value("momentY", position)?,
                bending_z_nm: value("momentZ", position)?,
            });
        }
        elements.push(FrameResponseRecoveryElement { element_id: element_id.clone(), stations });
    }
    elements.sort_by(|a, b| bytewise_utf8_compare(&a.element_id, &b.element_id));
    Ok(FrameResponseRecovery {
        schema_version: "FP_FRAME_RESPONSE_RECOVERY_RESULT_1".to_string(),
        request_id: compiled.request_id.clone(),
        elements,
        response_hash: "internal".to_string(),
    })
}
