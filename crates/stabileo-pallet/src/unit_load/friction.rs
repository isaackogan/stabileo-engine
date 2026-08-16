//! The Coulomb two-tangent rule at one interface.
//!
//! Literal port of `packages/analysis/unit-load/src/friction.ts`. The TS types
//! are plain records already (no zod, no tagged vectors), so the structs below
//! are field-for-field; the two string unions become enums whose serde forms
//! ARE the reference vocabulary (`STICK`/`SLIP`, `ZERO_NORMAL`/`WITHIN_CONE`/
//! `ON_CONE_TIE_STICK`/`PROJECTED_TO_CONE`), with `as_str` for callers that
//! need the word itself.

use serde::{Deserialize, Serialize};

use crate::types::{PalletError, PalletResult};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct CoulombState2TRequest {
    pub interface_id: String,
    pub trial_force_x_n: f64,
    pub trial_force_z_n: f64,
    pub normal_force_n: f64,
    pub friction_coefficient: f64,
    pub relative_movement_x_m: f64,
    pub relative_movement_z_m: f64,
    pub tolerance_n: f64,
}

/// TS `CoulombState2TResult["state"]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum TangentState {
    Stick,
    Slip,
}

impl TangentState {
    pub fn as_str(self) -> &'static str {
        match self {
            TangentState::Stick => "STICK",
            TangentState::Slip => "SLIP",
        }
    }
}

/// TS `CoulombState2TResult["decisionCode"]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum CoulombDecisionCode {
    ZeroNormal,
    WithinCone,
    OnConeTieStick,
    ProjectedToCone,
}

impl CoulombDecisionCode {
    pub fn as_str(self) -> &'static str {
        match self {
            CoulombDecisionCode::ZeroNormal => "ZERO_NORMAL",
            CoulombDecisionCode::WithinCone => "WITHIN_CONE",
            CoulombDecisionCode::OnConeTieStick => "ON_CONE_TIE_STICK",
            CoulombDecisionCode::ProjectedToCone => "PROJECTED_TO_CONE",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct CoulombState2TResult {
    pub interface_id: String,
    pub state: TangentState,
    pub tangent_force_x_n: f64,
    pub tangent_force_z_n: f64,
    pub slip_direction_x: f64,
    pub slip_direction_z: f64,
    pub friction_limit_n: f64,
    pub dissipated_energy_j: f64,
    pub decision_code: CoulombDecisionCode,
}

pub fn evaluate_coulomb_state_2t(
    request: &CoulombState2TRequest,
) -> PalletResult<CoulombState2TResult> {
    // TS: `Object.values(request).filter(value => typeof value === "number")`
    // — every field except `interfaceId`, which is the only string on the
    // record; the seven numbers below are exactly that set.
    let values = [
        request.trial_force_x_n,
        request.trial_force_z_n,
        request.normal_force_n,
        request.friction_coefficient,
        request.relative_movement_x_m,
        request.relative_movement_z_m,
        request.tolerance_n,
    ];
    if values.iter().any(|value| !value.is_finite()) {
        return Err(PalletError::sentence("NON_FINITE_FRICTION_INPUT"));
    }
    if request.normal_force_n < 0.0
        || request.friction_coefficient < 0.0
        || request.tolerance_n <= 0.0
    {
        return Err(PalletError::sentence("INVALID_FRICTION_INPUT"));
    }
    let trial = request.trial_force_x_n.hypot(request.trial_force_z_n);
    let limit = request.normal_force_n * request.friction_coefficient;
    if limit == 0.0 {
        return Ok(CoulombState2TResult {
            interface_id: request.interface_id.clone(),
            state: TangentState::Slip,
            tangent_force_x_n: 0.0,
            tangent_force_z_n: 0.0,
            slip_direction_x: 0.0,
            slip_direction_z: 0.0,
            friction_limit_n: 0.0,
            dissipated_energy_j: 0.0,
            decision_code: CoulombDecisionCode::ZeroNormal,
        });
    }
    if trial <= limit + request.tolerance_n {
        return Ok(CoulombState2TResult {
            interface_id: request.interface_id.clone(),
            state: TangentState::Stick,
            tangent_force_x_n: request.trial_force_x_n,
            tangent_force_z_n: request.trial_force_z_n,
            slip_direction_x: 0.0,
            slip_direction_z: 0.0,
            friction_limit_n: limit,
            dissipated_energy_j: 0.0,
            decision_code: if (trial - limit).abs() <= request.tolerance_n {
                CoulombDecisionCode::OnConeTieStick
            } else {
                CoulombDecisionCode::WithinCone
            },
        });
    }
    let movement = request.relative_movement_x_m.hypot(request.relative_movement_z_m);
    let direction_x = if movement > 0.0 {
        request.relative_movement_x_m / movement
    } else {
        request.trial_force_x_n / trial
    };
    let direction_z = if movement > 0.0 {
        request.relative_movement_z_m / movement
    } else {
        request.trial_force_z_n / trial
    };
    Ok(CoulombState2TResult {
        interface_id: request.interface_id.clone(),
        state: TangentState::Slip,
        tangent_force_x_n: -limit * direction_x,
        tangent_force_z_n: -limit * direction_z,
        slip_direction_x: direction_x,
        slip_direction_z: direction_z,
        friction_limit_n: limit,
        dissipated_energy_j: limit * movement,
        decision_code: CoulombDecisionCode::ProjectedToCone,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn request() -> CoulombState2TRequest {
        CoulombState2TRequest {
            interface_id: "i".to_string(),
            trial_force_x_n: 0.0,
            trial_force_z_n: 0.0,
            normal_force_n: 100.0,
            friction_coefficient: 0.5,
            relative_movement_x_m: 0.0,
            relative_movement_z_m: 0.0,
            tolerance_n: 0.01,
        }
    }

    /// limit = 100·0.5 = 50; trial = hypot(10,0) = 10 ≤ 50.01 → STICK, and
    /// |10 − 50| = 40 > 0.01 → WITHIN_CONE. The trial force passes through
    /// untouched, with no slip direction and no dissipation.
    #[test]
    fn stick_inside_the_cone_passes_the_trial_force_through() {
        let result = evaluate_coulomb_state_2t(&CoulombState2TRequest {
            trial_force_x_n: 10.0,
            ..request()
        })
        .expect("finite request");
        assert_eq!(result.state, TangentState::Stick);
        assert_eq!(result.decision_code, CoulombDecisionCode::WithinCone);
        assert_eq!(result.tangent_force_x_n, 10.0);
        assert_eq!(result.tangent_force_z_n, 0.0);
        assert_eq!(result.slip_direction_x, 0.0);
        assert_eq!(result.slip_direction_z, 0.0);
        assert_eq!(result.friction_limit_n, 50.0);
        assert_eq!(result.dissipated_energy_j, 0.0);
        assert_eq!(result.interface_id, "i");
    }

    /// trial = 50 exactly on the limit: still ≤ 50.01 → STICK, and |50 − 50| =
    /// 0 ≤ 0.01 → the tie code. A trial 30/40 → hypot 50 lands the same way,
    /// which also pins that the cone is measured on the RESULTANT tangent.
    #[test]
    fn stick_on_the_cone_reports_the_tie() {
        let axial = evaluate_coulomb_state_2t(&CoulombState2TRequest {
            trial_force_x_n: 50.0,
            ..request()
        })
        .expect("finite request");
        assert_eq!(axial.state, TangentState::Stick);
        assert_eq!(axial.decision_code, CoulombDecisionCode::OnConeTieStick);
        let diagonal = evaluate_coulomb_state_2t(&CoulombState2TRequest {
            trial_force_x_n: 30.0,
            trial_force_z_n: 40.0,
            ..request()
        })
        .expect("finite request");
        assert_eq!(diagonal.state, TangentState::Stick);
        assert_eq!(diagonal.decision_code, CoulombDecisionCode::OnConeTieStick);
        assert_eq!(diagonal.tangent_force_x_n, 30.0);
        assert_eq!(diagonal.tangent_force_z_n, 40.0);
    }

    /// trial = 60 > 50.01, movement = hypot(0,−2) = 2 → direction (0/2, −2/2)
    /// = (0,−1). The force is the limit PUSHED BACK along the movement:
    /// (−50·0, −50·(−1)) = (−0, 50); dissipation = 50·2 = 100. Every value is
    /// exact in f64.
    #[test]
    fn slip_projects_onto_the_cone_along_the_measured_movement() {
        let result = evaluate_coulomb_state_2t(&CoulombState2TRequest {
            trial_force_x_n: 60.0,
            relative_movement_z_m: -2.0,
            ..request()
        })
        .expect("finite request");
        assert_eq!(result.state, TangentState::Slip);
        assert_eq!(result.decision_code, CoulombDecisionCode::ProjectedToCone);
        assert_eq!(result.slip_direction_x, 0.0);
        assert_eq!(result.slip_direction_z, -1.0);
        assert_eq!(result.tangent_force_x_n, 0.0);
        assert_eq!(result.tangent_force_z_n, 50.0);
        assert_eq!(result.friction_limit_n, 50.0);
        assert_eq!(result.dissipated_energy_j, 100.0);
    }

    /// Nothing has moved yet: the direction falls back to the trial force's
    /// own, trial = hypot(60,80) = 100 → (0.6, 0.8), and the dissipation is
    /// limit·0 = 0. The expectations are written as the reference's own
    /// expressions because 3/5 and 4/5 are not dyadic.
    #[test]
    fn slip_without_movement_falls_back_to_the_trial_direction() {
        let result = evaluate_coulomb_state_2t(&CoulombState2TRequest {
            trial_force_x_n: 60.0,
            trial_force_z_n: 80.0,
            ..request()
        })
        .expect("finite request");
        assert_eq!(result.state, TangentState::Slip);
        assert_eq!(result.decision_code, CoulombDecisionCode::ProjectedToCone);
        assert_eq!(result.slip_direction_x, 60.0 / 100.0);
        assert_eq!(result.slip_direction_z, 80.0 / 100.0);
        assert_eq!(result.tangent_force_x_n, -50.0 * (60.0 / 100.0));
        assert_eq!(result.tangent_force_z_n, -50.0 * (80.0 / 100.0));
        assert_eq!(result.dissipated_energy_j, 0.0);
    }

    /// No normal force → no cone at all: the interface is reported SLIP with
    /// everything zeroed. A zero friction coefficient under load takes the
    /// same branch, because the branch tests the PRODUCT.
    #[test]
    fn zero_normal_force_reports_slip_with_no_capacity() {
        let no_normal = evaluate_coulomb_state_2t(&CoulombState2TRequest {
            trial_force_x_n: 1.0,
            trial_force_z_n: 1.0,
            normal_force_n: 0.0,
            ..request()
        })
        .expect("finite request");
        assert_eq!(no_normal.state, TangentState::Slip);
        assert_eq!(no_normal.decision_code, CoulombDecisionCode::ZeroNormal);
        assert_eq!(no_normal.tangent_force_x_n, 0.0);
        assert_eq!(no_normal.tangent_force_z_n, 0.0);
        assert_eq!(no_normal.slip_direction_x, 0.0);
        assert_eq!(no_normal.slip_direction_z, 0.0);
        assert_eq!(no_normal.friction_limit_n, 0.0);
        assert_eq!(no_normal.dissipated_energy_j, 0.0);

        let no_friction = evaluate_coulomb_state_2t(&CoulombState2TRequest {
            trial_force_x_n: 1.0,
            friction_coefficient: 0.0,
            ..request()
        })
        .expect("finite request");
        assert_eq!(no_friction.decision_code, CoulombDecisionCode::ZeroNormal);
        assert_eq!(no_friction.friction_limit_n, 0.0);
    }

    #[test]
    fn non_finite_inputs_are_refused_before_the_range_checks() {
        for request in [
            CoulombState2TRequest { trial_force_x_n: f64::NAN, ..request() },
            CoulombState2TRequest { relative_movement_z_m: f64::INFINITY, ..request() },
            // Negative AND non-finite: the finiteness guard runs first, so the
            // code is the non-finite one.
            CoulombState2TRequest { normal_force_n: f64::NEG_INFINITY, ..request() },
        ] {
            let error = evaluate_coulomb_state_2t(&request).expect_err("refused");
            assert_eq!(error.code, "NON_FINITE_FRICTION_INPUT");
            assert_eq!(error.message, "NON_FINITE_FRICTION_INPUT");
        }
    }

    #[test]
    fn out_of_range_inputs_are_refused() {
        for request in [
            CoulombState2TRequest { normal_force_n: -1.0, ..request() },
            CoulombState2TRequest { friction_coefficient: -0.1, ..request() },
            CoulombState2TRequest { tolerance_n: 0.0, ..request() },
            CoulombState2TRequest { tolerance_n: -0.01, ..request() },
        ] {
            let error = evaluate_coulomb_state_2t(&request).expect_err("refused");
            assert_eq!(error.code, "INVALID_FRICTION_INPUT");
        }
    }
}
