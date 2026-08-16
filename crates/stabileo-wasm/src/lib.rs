//! The one shipped WASM binary. Linking the kernel crate re-exports its
//! entire `#[wasm_bindgen]` surface unchanged — every existing operation
//! keeps its name and signature — and the pallet orchestration adds its own
//! entries beside them.

use wasm_bindgen::prelude::*;

pub use dedaliano_engine::*;

use serde::Deserialize;
use stabileo_pallet::coupled::{solve_coupled_event, ProgressEmission};
use stabileo_pallet::native_port::NativeKernelPort;
use stabileo_pallet::pallet::projection::PalletMemberMap;
use stabileo_pallet::schema::{
    AnalysisFrame, CompiledPalletSupportState, NumericalAcceptanceProfile, UnitLoadActiveState,
};

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct CoupledEventRequest {
    base_frame: AnalysisFrame,
    member_map: PalletMemberMap,
    support_state: CompiledPalletSupportState,
    unit_state: UnitLoadActiveState,
    #[serde(rename = "palletOverallM")]
    pallet_overall_m: OverallM,
    numerical_profile: NumericalAcceptanceProfile,
    event_id: String,
}

#[derive(Deserialize)]
struct OverallM {
    length: f64,
    width: f64,
}

/// ONE CALL, ONE LOAD EVENT, SOLVED TO CONVERGENCE.
///
/// The entire coupled pallet/unit-load analysis — the unilateral floor
/// contact search with Coulomb stick/slip and the bistable freeze, the
/// rigid-body load partition with measured deck compliance, the contact
/// projection and top-response recovery, and the coupled fixed-point loop —
/// runs inside this call, invoking the kernel natively per inner round with
/// nothing serialized between rounds. JSON crosses the boundary exactly
/// twice: the compiled inputs in, the converged terminal state out.
///
/// `progress`, when provided, receives one JSON string per emission:
/// `{"kind":"phase", phase, couplingRound, total, shareResidual,
/// translationResidualM, rotationResidualRad}` per kernel call, and
/// `{"kind":"note", message}` for the walk's disclosure lines (bistable
/// freezes). Callback errors are swallowed — progress is a window, never a
/// participant.
///
/// Failures return the orchestration's own diagnostic sentence (the same
/// NON_CONVERGED / PALLET_RESPONSE_IMPLAUSIBLE / KERNEL_FAILURE sentences
/// the reference threw), carrying the round, the support census, and the
/// kernel's own words.
#[wasm_bindgen]
pub fn solve_coupled_pallet_event(
    request_json: &str,
    progress: Option<js_sys::Function>,
) -> Result<String, JsValue> {
    let request: CoupledEventRequest = serde_json::from_str(request_json)
        .map_err(|error| JsValue::from_str(&format!("INPUT_INVALID: {error}")))?;
    let mut kernel = NativeKernelPort::new(request.numerical_profile.clone());
    let mut sink = |emission: &ProgressEmission| {
        if let Some(callback) = &progress {
            if let Ok(payload) = serde_json::to_string(emission) {
                let _ = callback.call1(&JsValue::NULL, &JsValue::from_str(&payload));
            }
        }
    };
    let result = solve_coupled_event(
        &mut kernel,
        &request.base_frame,
        &request.member_map,
        &request.support_state,
        &request.unit_state,
        (request.pallet_overall_m.length, request.pallet_overall_m.width),
        &request.numerical_profile,
        &request.event_id,
        &mut sink,
    )
    .map_err(|error| JsValue::from_str(&error.message))?;
    serde_json::to_string(&result)
        .map_err(|error| JsValue::from_str(&format!("CORRUPT_OUTPUT: {error}")))
}
