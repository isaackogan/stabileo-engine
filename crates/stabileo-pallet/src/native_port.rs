//! The native binding of the coupled loop's [`KernelPort`] seam onto the
//! kernel bridge: compile → solve → normalize per round, with the compiled
//! model and raw results held for terminal station recovery — the in-process
//! replacement for the TS kernel class's result-hash cache, which existed
//! only to bridge two WASM calls.

use crate::coupled::KernelPort;
use crate::kernel_bridge::recovery::FrameResponseStationCount;
use crate::kernel_bridge::{
    recover_frame_response, solve_static_active_state_detailed, FrameResponseRecovery, StaticSolve,
};
use crate::schema::{AnalysisFrame, KernelResult, NumericalAcceptanceProfile};
use crate::types::{PalletError, PalletResult};

pub struct NativeKernelPort {
    profile: NumericalAcceptanceProfile,
    last: Option<StaticSolve>,
}

impl NativeKernelPort {
    pub fn new(profile: NumericalAcceptanceProfile) -> Self {
        NativeKernelPort { profile, last: None }
    }
}

impl KernelPort for NativeKernelPort {
    fn solve(
        &mut self,
        frame: &AnalysisFrame,
        request_id: &str,
        active_state_id: &str,
    ) -> PalletResult<KernelResult> {
        let solve = solve_static_active_state_detailed(frame, &self.profile, request_id, active_state_id)?;
        let result = solve.result.clone();
        self.last = Some(solve);
        Ok(result)
    }

    fn recover(
        &mut self,
        frame: &AnalysisFrame,
        kernel_result: &KernelResult,
        stations_per_element: u32,
    ) -> PalletResult<FrameResponseRecovery> {
        let last = self.last.as_ref().ok_or_else(|| {
            PalletError::sentence("RECOVERY_RESULT_NOT_CACHED: solve and recovery must use the same exact kernel result")
        })?;
        if last.result.request_id != kernel_result.request_id {
            return Err(PalletError::sentence(
                "RECOVERY_RESULT_NOT_CACHED: solve and recovery must use the same exact kernel result",
            ));
        }
        let station_counts: Vec<FrameResponseStationCount> = frame
            .elements
            .iter()
            .map(|element| FrameResponseStationCount {
                element_id: element.element_id.clone(),
                count: stations_per_element as usize,
            })
            .collect();
        recover_frame_response(&last.compiled, &last.raw, &station_counts)
    }
}
