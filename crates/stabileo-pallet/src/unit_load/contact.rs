//! The unilateral normal-contact rule at one interface.
//!
//! Literal port of `packages/analysis/unit-load/src/contact.ts`. The TS types
//! are plain records (no zod, no tagged vectors), so the structs are
//! field-for-field; the two string unions become enums whose serde forms ARE
//! the reference vocabulary (`OPEN`/`CLOSED`, `GAP_OPEN`/`COMPRESSION_CLOSE`/
//! `TIE_CLOSE`).

use serde::{Deserialize, Serialize};

use crate::types::{PalletError, PalletResult};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct NormalContactRequest {
    pub interface_id: String,
    pub gap_m: f64,
    pub trial_normal_force_n: f64,
    pub tolerance_n: f64,
}

/// TS `NormalContactResult["state"]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum NormalState {
    Open,
    Closed,
}

impl NormalState {
    pub fn as_str(self) -> &'static str {
        match self {
            NormalState::Open => "OPEN",
            NormalState::Closed => "CLOSED",
        }
    }
}

/// TS `NormalContactResult["decisionCode"]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum NormalContactDecisionCode {
    GapOpen,
    CompressionClose,
    TieClose,
}

impl NormalContactDecisionCode {
    pub fn as_str(self) -> &'static str {
        match self {
            NormalContactDecisionCode::GapOpen => "GAP_OPEN",
            NormalContactDecisionCode::CompressionClose => "COMPRESSION_CLOSE",
            NormalContactDecisionCode::TieClose => "TIE_CLOSE",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct NormalContactResult {
    pub interface_id: String,
    pub state: NormalState,
    pub normal_force_n: f64,
    pub gap_m: f64,
    pub complementarity_residual_n: f64,
    pub decision_code: NormalContactDecisionCode,
}

pub fn evaluate_normal_contact(
    request: &NormalContactRequest,
) -> PalletResult<NormalContactResult> {
    for value in [request.gap_m, request.trial_normal_force_n, request.tolerance_n] {
        if !value.is_finite() {
            return Err(PalletError::sentence("NON_FINITE_CONTACT_INPUT"));
        }
    }
    if request.tolerance_n <= 0.0 {
        return Err(PalletError::sentence("INVALID_CONTACT_TOLERANCE"));
    }
    if request.gap_m > 0.0 {
        return Ok(NormalContactResult {
            interface_id: request.interface_id.clone(),
            state: NormalState::Open,
            normal_force_n: 0.0,
            gap_m: request.gap_m,
            // TS `Math.max(0, trialNormalForceN)`.
            complementarity_residual_n: js_max(0.0, request.trial_normal_force_n),
            decision_code: NormalContactDecisionCode::GapOpen,
        });
    }
    if request.trial_normal_force_n > request.tolerance_n {
        return Ok(NormalContactResult {
            interface_id: request.interface_id.clone(),
            state: NormalState::Closed,
            normal_force_n: request.trial_normal_force_n,
            gap_m: request.gap_m,
            // TS `Math.abs(Math.min(0, gapM)) * trialNormalForceN`.
            complementarity_residual_n: js_min(0.0, request.gap_m).abs()
                * request.trial_normal_force_n,
            decision_code: NormalContactDecisionCode::CompressionClose,
        });
    }
    Ok(NormalContactResult {
        interface_id: request.interface_id.clone(),
        state: NormalState::Closed,
        normal_force_n: js_max(0.0, request.trial_normal_force_n),
        gap_m: request.gap_m,
        complementarity_residual_n: request.trial_normal_force_n.abs(),
        decision_code: NormalContactDecisionCode::TieClose,
    })
}

/// `Math.max` on two numbers, JS semantics: NaN is contagious (Rust's
/// `f64::max` swallows it instead), and +0 wins over −0. The finiteness guard
/// above makes the NaN half unreachable from this module, but the shape of the
/// reference is what is being ported.
fn js_max(left: f64, right: f64) -> f64 {
    if left.is_nan() || right.is_nan() {
        return f64::NAN;
    }
    if right > left {
        right
    } else {
        left
    }
}

/// `Math.min` on two numbers, JS semantics (see [`js_max`]).
fn js_min(left: f64, right: f64) -> f64 {
    if left.is_nan() || right.is_nan() {
        return f64::NAN;
    }
    if right < left {
        right
    } else {
        left
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn request() -> NormalContactRequest {
        NormalContactRequest {
            interface_id: "i".to_string(),
            gap_m: 0.0,
            trial_normal_force_n: 0.0,
            tolerance_n: 0.1,
        }
    }

    /// A positive gap is an open contact: it carries nothing, and the
    /// complementarity residual is whatever compression the trial wanted to
    /// put through the hole — max(0, 100) = 100. A trial that PULLS leaves no
    /// residual at all: max(0, −100) = 0.
    #[test]
    fn a_positive_gap_opens_the_contact() {
        let compressing = evaluate_normal_contact(&NormalContactRequest {
            gap_m: 0.001,
            trial_normal_force_n: 100.0,
            ..request()
        })
        .expect("finite request");
        assert_eq!(compressing.state, NormalState::Open);
        assert_eq!(compressing.decision_code, NormalContactDecisionCode::GapOpen);
        assert_eq!(compressing.normal_force_n, 0.0);
        assert_eq!(compressing.gap_m, 0.001);
        assert_eq!(compressing.complementarity_residual_n, 100.0);

        let pulling = evaluate_normal_contact(&NormalContactRequest {
            gap_m: 0.001,
            trial_normal_force_n: -100.0,
            ..request()
        })
        .expect("finite request");
        assert_eq!(pulling.complementarity_residual_n, 0.0);
    }

    /// Closed and compressed: the trial force passes through, and the residual
    /// is the penetration times the force — |min(0, −0.5)|·100 = 50.
    #[test]
    fn compression_beyond_the_tolerance_closes_the_contact() {
        let result = evaluate_normal_contact(&NormalContactRequest {
            gap_m: -0.5,
            trial_normal_force_n: 100.0,
            ..request()
        })
        .expect("finite request");
        assert_eq!(result.state, NormalState::Closed);
        assert_eq!(result.decision_code, NormalContactDecisionCode::CompressionClose);
        assert_eq!(result.normal_force_n, 100.0);
        assert_eq!(result.gap_m, -0.5);
        assert_eq!(result.complementarity_residual_n, 50.0);
    }

    /// The tie: touching (gap 0) with a trial force at or below the tolerance.
    /// The contact stays CLOSED, carries max(0, trial), and reports |trial| as
    /// the residual — so a pulling trial of −5 closes at zero force with a
    /// residual of 5.
    #[test]
    fn a_trial_within_the_tolerance_ties_closed() {
        let touching = evaluate_normal_contact(&NormalContactRequest {
            gap_m: 0.0,
            trial_normal_force_n: 0.0,
            ..request()
        })
        .expect("finite request");
        assert_eq!(touching.state, NormalState::Closed);
        assert_eq!(touching.decision_code, NormalContactDecisionCode::TieClose);
        assert_eq!(touching.normal_force_n, 0.0);
        assert_eq!(touching.complementarity_residual_n, 0.0);

        let pulling = evaluate_normal_contact(&NormalContactRequest {
            gap_m: 0.0,
            trial_normal_force_n: -5.0,
            ..request()
        })
        .expect("finite request");
        assert_eq!(pulling.decision_code, NormalContactDecisionCode::TieClose);
        assert_eq!(pulling.normal_force_n, 0.0);
        assert_eq!(pulling.complementarity_residual_n, 5.0);

        // Exactly AT the tolerance is a tie, not a compression: the branch
        // tests `trial > tolerance`.
        let at_tolerance = evaluate_normal_contact(&NormalContactRequest {
            gap_m: 0.0,
            trial_normal_force_n: 0.1,
            ..request()
        })
        .expect("finite request");
        assert_eq!(at_tolerance.decision_code, NormalContactDecisionCode::TieClose);
        assert_eq!(at_tolerance.normal_force_n, 0.1);
        assert_eq!(at_tolerance.complementarity_residual_n, 0.1);
    }

    #[test]
    fn non_finite_inputs_are_refused() {
        for request in [
            NormalContactRequest { gap_m: f64::NAN, ..request() },
            NormalContactRequest { trial_normal_force_n: f64::INFINITY, ..request() },
            NormalContactRequest { tolerance_n: f64::NAN, ..request() },
        ] {
            let error = evaluate_normal_contact(&request).expect_err("refused");
            assert_eq!(error.code, "NON_FINITE_CONTACT_INPUT");
            assert_eq!(error.message, "NON_FINITE_CONTACT_INPUT");
        }
    }

    #[test]
    fn a_non_positive_tolerance_is_refused() {
        for tolerance in [0.0, -0.1] {
            let error =
                evaluate_normal_contact(&NormalContactRequest { tolerance_n: tolerance, ..request() })
                    .expect_err("refused");
            assert_eq!(error.code, "INVALID_CONTACT_TOLERANCE");
        }
    }
}
