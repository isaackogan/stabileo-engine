//! The unit-load capacity criterion: demand over capacity, classified.
//!
//! Literal port of `packages/analysis/unit-load/src/criteria.ts`.
//!
//! SHAPE. The TS return type is a zod UNION of three record shapes that agree
//! on every field except two: the finite-capacity member carries a numeric
//! `utilization` and NO `reasonCode`, while the two zero-capacity members
//! carry `utilization: null` and a `reasonCode`. One struct with
//! `utilization: Option<f64>` and `reason_code: Option<…>` reproduces all
//! three exactly — `utilization` serialises as `null` when absent (the union
//! spells the null out), `reasonCode` is omitted entirely when absent (the
//! union's finite member is `.strict()` and would reject the key). The zod
//! `.parse()` at the return is validation only and is skipped per the porting
//! rules.

use serde::{Deserialize, Serialize};

use crate::types::{PalletError, PalletResult};

/// TS `UnitLoadCriterionKind` — what the unit-load engine can actually compute
/// from geometry, mass and friction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum UnitLoadCriterionKind {
    Sliding,
    Overturning,
    InterlayerShear,
}

impl UnitLoadCriterionKind {
    pub fn as_str(self) -> &'static str {
        match self {
            UnitLoadCriterionKind::Sliding => "SLIDING",
            UnitLoadCriterionKind::Overturning => "OVERTURNING",
            UnitLoadCriterionKind::InterlayerShear => "INTERLAYER_SHEAR",
        }
    }
}

/// TS `UnitLoadCriterionResult["classification"]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum CriterionClassification {
    Pass,
    Review,
    Fail,
}

impl CriterionClassification {
    pub fn as_str(self) -> &'static str {
        match self {
            CriterionClassification::Pass => "PASS",
            CriterionClassification::Review => "REVIEW",
            CriterionClassification::Fail => "FAIL",
        }
    }
}

/// TS `UnitLoadCriterionResult["reasonCode"]`, present only on the two
/// zero-capacity members of the union.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum CriterionReasonCode {
    ZeroDemandZeroCapacity,
    ZeroCapacityWithPositiveDemand,
}

impl CriterionReasonCode {
    pub fn as_str(self) -> &'static str {
        match self {
            CriterionReasonCode::ZeroDemandZeroCapacity => "ZERO_DEMAND_ZERO_CAPACITY",
            CriterionReasonCode::ZeroCapacityWithPositiveDemand => {
                "ZERO_CAPACITY_WITH_POSITIVE_DEMAND"
            }
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct EvaluateUnitLoadCapacityCriterionRequest {
    pub criterion_id: String,
    pub kind: UnitLoadCriterionKind,
    pub demand: f64,
    pub capacity: f64,
    pub governing_entity_id: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct UnitLoadCriterionResult {
    pub criterion_id: String,
    pub kind: UnitLoadCriterionKind,
    pub classification: CriterionClassification,
    pub demand: f64,
    pub capacity: f64,
    pub utilization: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reason_code: Option<CriterionReasonCode>,
    pub governing_entity_id: String,
}

pub fn evaluate_unit_load_capacity_criterion(
    request: &EvaluateUnitLoadCapacityCriterionRequest,
) -> PalletResult<UnitLoadCriterionResult> {
    if ![request.demand, request.capacity].iter().all(|value| value.is_finite()) {
        return Err(PalletError::sentence("NON_FINITE_UNIT_LOAD_CRITERION_INPUT"));
    }
    if request.demand < 0.0 || request.capacity < 0.0 {
        return Err(PalletError::sentence("NEGATIVE_UNIT_LOAD_CRITERION_INPUT"));
    }
    // TS `common` — the fields every member of the union shares, spread into
    // whichever member the branches below select.
    let common = |classification: CriterionClassification,
                  utilization: Option<f64>,
                  reason_code: Option<CriterionReasonCode>| UnitLoadCriterionResult {
        criterion_id: request.criterion_id.clone(),
        kind: request.kind,
        classification,
        demand: request.demand,
        capacity: request.capacity,
        utilization,
        reason_code,
        governing_entity_id: request.governing_entity_id.clone(),
    };
    if request.capacity == 0.0 {
        return Ok(if request.demand == 0.0 {
            common(
                CriterionClassification::Pass,
                None,
                Some(CriterionReasonCode::ZeroDemandZeroCapacity),
            )
        } else {
            common(
                CriterionClassification::Fail,
                None,
                Some(CriterionReasonCode::ZeroCapacityWithPositiveDemand),
            )
        });
    }
    let utilization = request.demand / request.capacity;
    Ok(common(
        if utilization > 1.0 {
            CriterionClassification::Fail
        } else if utilization > 0.9 {
            CriterionClassification::Review
        } else {
            CriterionClassification::Pass
        },
        Some(utilization),
        None,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn request(demand: f64, capacity: f64) -> EvaluateUnitLoadCapacityCriterionRequest {
        EvaluateUnitLoadCapacityCriterionRequest {
            criterion_id: "c".to_string(),
            kind: UnitLoadCriterionKind::Sliding,
            demand,
            capacity,
            governing_entity_id: "PALLETIZED_BOXES".to_string(),
        }
    }

    /// 5/10 = 0.5 ≤ 0.9 → PASS, and every common field is carried through.
    #[test]
    fn a_utilization_under_nine_tenths_passes() {
        let result =
            evaluate_unit_load_capacity_criterion(&request(5.0, 10.0)).expect("finite request");
        assert_eq!(result.classification, CriterionClassification::Pass);
        assert_eq!(result.utilization, Some(0.5));
        assert_eq!(result.reason_code, None);
        assert_eq!(result.criterion_id, "c");
        assert_eq!(result.kind, UnitLoadCriterionKind::Sliding);
        assert_eq!(result.demand, 5.0);
        assert_eq!(result.capacity, 10.0);
        assert_eq!(result.governing_entity_id, "PALLETIZED_BOXES");
    }

    /// The two thresholds are strict `>`: 0.9 exactly still PASSES, 0.95
    /// REVIEWS, 1.0 exactly still REVIEWS (it is not > 1), and anything above
    /// 1 FAILS.
    #[test]
    fn the_classification_thresholds_are_strict() {
        let at_nine_tenths =
            evaluate_unit_load_capacity_criterion(&request(9.0, 10.0)).expect("finite request");
        assert_eq!(at_nine_tenths.classification, CriterionClassification::Pass);
        assert_eq!(at_nine_tenths.utilization, Some(9.0 / 10.0));

        let review =
            evaluate_unit_load_capacity_criterion(&request(9.5, 10.0)).expect("finite request");
        assert_eq!(review.classification, CriterionClassification::Review);
        assert_eq!(review.utilization, Some(9.5 / 10.0));

        let at_unity =
            evaluate_unit_load_capacity_criterion(&request(10.0, 10.0)).expect("finite request");
        assert_eq!(at_unity.classification, CriterionClassification::Review);
        assert_eq!(at_unity.utilization, Some(1.0));

        let fail =
            evaluate_unit_load_capacity_criterion(&request(11.0, 10.0)).expect("finite request");
        assert_eq!(fail.classification, CriterionClassification::Fail);
        assert_eq!(fail.utilization, Some(11.0 / 10.0));
    }

    /// Zero over zero is not a division: no demand against no capacity passes
    /// with no utilization to report, and any demand against no capacity
    /// fails the same way.
    #[test]
    fn zero_capacity_is_classified_without_dividing() {
        let vacuous =
            evaluate_unit_load_capacity_criterion(&request(0.0, 0.0)).expect("finite request");
        assert_eq!(vacuous.classification, CriterionClassification::Pass);
        assert_eq!(vacuous.utilization, None);
        assert_eq!(vacuous.reason_code, Some(CriterionReasonCode::ZeroDemandZeroCapacity));

        let starved =
            evaluate_unit_load_capacity_criterion(&request(5.0, 0.0)).expect("finite request");
        assert_eq!(starved.classification, CriterionClassification::Fail);
        assert_eq!(starved.utilization, None);
        assert_eq!(
            starved.reason_code,
            Some(CriterionReasonCode::ZeroCapacityWithPositiveDemand)
        );
    }

    #[test]
    fn non_finite_and_negative_inputs_are_refused() {
        for (demand, capacity) in [(f64::NAN, 10.0), (5.0, f64::INFINITY)] {
            let error = evaluate_unit_load_capacity_criterion(&request(demand, capacity))
                .expect_err("refused");
            assert_eq!(error.code, "NON_FINITE_UNIT_LOAD_CRITERION_INPUT");
            assert_eq!(error.message, "NON_FINITE_UNIT_LOAD_CRITERION_INPUT");
        }
        for (demand, capacity) in [(-1.0, 10.0), (5.0, -10.0)] {
            let error = evaluate_unit_load_capacity_criterion(&request(demand, capacity))
                .expect_err("refused");
            assert_eq!(error.code, "NEGATIVE_UNIT_LOAD_CRITERION_INPUT");
        }
    }

    /// The finite member must not carry a `reasonCode` key at all (the TS
    /// member is `.strict()`), while the zero-capacity members spell the null
    /// utilization out.
    #[test]
    fn the_serialised_shape_matches_the_union_members() {
        let finite =
            evaluate_unit_load_capacity_criterion(&request(5.0, 10.0)).expect("finite request");
        let json = serde_json::to_value(&finite).expect("serialises");
        assert!(json.get("reasonCode").is_none());
        assert_eq!(json["utilization"], serde_json::json!(0.5));
        assert_eq!(json["classification"], serde_json::json!("PASS"));
        assert_eq!(json["kind"], serde_json::json!("SLIDING"));
        assert_eq!(json["criterionId"], serde_json::json!("c"));
        assert_eq!(json["governingEntityId"], serde_json::json!("PALLETIZED_BOXES"));

        let vacuous =
            evaluate_unit_load_capacity_criterion(&request(0.0, 0.0)).expect("finite request");
        let json = serde_json::to_value(&vacuous).expect("serialises");
        assert_eq!(json["utilization"], serde_json::Value::Null);
        assert_eq!(json["reasonCode"], serde_json::json!("ZERO_DEMAND_ZERO_CAPACITY"));
    }
}
