//! The kernel bridge: the application's TypeScript adapter for the Dedaliano
//! structural kernel, ported literally, INSIDE the engine.
//!
//! `packages/analysis/stabileo` compiles an `AnalysisFrame` into the kernel's
//! solver input, calls the WASM boundary, and normalizes what comes back into
//! the application's `KernelResult`, refusing any answer that fails an
//! equilibrium audit. This module is that pipeline with the boundary removed:
//! the same compile, the same solve, the same audit, the same sentences — but
//! the solver input is built as native structs and handed straight to
//! `solver::linear::solve_3d` / `solver::constraints::solve_constrained_3d`,
//! so the coupled loop can iterate without serializing anything.
//!
//! What the TS does that this does NOT (PORTING.md rule 7): the per-call
//! `sha256CanonicalSync` identity stamps. `activeStateHash` and `resultHash`
//! are carried as `"internal"`; the application re-derives its own identity on
//! the terminal state it gets back.

pub mod compile;
pub mod coordinates;
pub mod diagnostics;
pub mod equilibrium;
pub mod id_map;
pub mod normalize;
pub mod number_format;
pub mod recovery;
pub mod units;

#[cfg(test)]
mod tests;

use dedaliano_engine::solver::constraints::{solve_constrained_3d, ConstrainedInput3D};
use dedaliano_engine::solver::linear::solve_3d;
use dedaliano_engine::types::AnalysisResults3D;

use crate::schema::{AnalysisFrame, KernelResult, NumericalAcceptanceProfile};
use crate::types::{PalletError, PalletResult};

pub use compile::{compile_static_active_state, CompiledStabileoModel};
pub use diagnostics::{classify_engine_failure, ClassifiedEngineFailure};
pub use equilibrium::{audit_equilibrium, EquilibriumAudit};
pub use normalize::{normalize_static_result, ConnectorResponse, ConstraintForceGroup};
pub use recovery::{
    recover_frame_response, FrameResponseRecovery, FrameResponseRecoveryElement,
    FrameResponseStation, FrameResponseStationCount,
};

/// The placeholder the TS carries when a caller has no active-state identity
/// to stamp. PORTING.md rule 7: the loop computes no hashes, so both the
/// active-state hash and the result hash echo this.
pub const INTERNAL_IDENTITY: &str = "internal";

/// One solve, with everything the caller needs to recover stations from it
/// without solving again.
///
/// The TS keeps a `Map<resultHash, {compiled, raw}>` inside the kernel class
/// so that `recoverFrameResponse3D` — a second WASM call — can find the model
/// its result came from. Natively there is no second call and no cache: the
/// compiled model and the raw results are right here.
#[derive(Debug, Clone)]
pub struct StaticSolve {
    pub compiled: CompiledStabileoModel,
    pub raw: AnalysisResults3D,
    pub result: KernelResult,
}

/// TS `StabileoStructuralKernel.solveStaticActiveState3D`.
///
/// PORT-QUESTION: `profile` is unused, and that is the faithful reading. The
/// TS audit (`equilibrium.ts`) hard-codes its own acceptance — a 1 mN absolute
/// floor and a reviewed 0.001 relative fraction — and takes only the frame's
/// characteristic length from the model; it never reads
/// `NumericalAcceptanceProfile.forceToleranceN`/`momentToleranceNm`, even
/// though the refusal sentence says "the frozen numerical profile". The
/// parameter is on the native signature by request and is carried, not
/// consulted. If the intent is for the profile's tolerances to GOVERN the
/// audit, that is a behaviour change and belongs in `equilibrium.rs` with the
/// TS changed alongside.
pub fn solve_static_active_state(
    frame: &AnalysisFrame,
    profile: &NumericalAcceptanceProfile,
    request_id: &str,
    active_state_id: &str,
) -> PalletResult<KernelResult> {
    solve_static_active_state_detailed(frame, profile, request_id, active_state_id)
        .map(|solve| solve.result)
}

/// The same solve, keeping the compiled model and the raw kernel results for
/// `recover_frame_response`.
pub fn solve_static_active_state_detailed(
    frame: &AnalysisFrame,
    profile: &NumericalAcceptanceProfile,
    request_id: &str,
    active_state_id: &str,
) -> PalletResult<StaticSolve> {
    // Carried, not consulted — see the PORT-QUESTION above.
    let _ = profile;
    let compiled = compile_static_active_state(
        frame,
        request_id,
        active_state_id,
        INTERNAL_IDENTITY,
    )?;
    let raw = solve_compiled(&compiled)?;
    let result = normalize_static_result(&compiled, &raw)?;
    Ok(StaticSolve { compiled, raw, result })
}

/// The kernel call itself: constrained when the frame has constraints, plain
/// linear otherwise, exactly as the TS chooses between `solveConstrained3D`
/// and `solve3D`.
///
/// A kernel failure is classified the way `classifyEngineFailure` classifies
/// it, so the code on the error is the same string the application's
/// `StabileoKernelError` carried. The `retryable` flag that error also carried
/// has nowhere to live on `PalletError`; call `classify_engine_failure` on the
/// message to recover it.
pub fn solve_compiled(compiled: &CompiledStabileoModel) -> PalletResult<AnalysisResults3D> {
    let solved = if !compiled.constraints.is_empty() {
        solve_constrained_3d(&ConstrainedInput3D {
            solver: compiled.input.clone(),
            constraints: compiled.constraints.clone(),
        })
    } else {
        solve_3d(&compiled.input)
    };
    solved.map_err(|error| {
        let failure = classify_engine_failure(&error);
        PalletError::new(failure.code, failure.message)
    })
}
