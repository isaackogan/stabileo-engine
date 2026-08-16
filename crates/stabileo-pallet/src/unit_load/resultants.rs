//! Resultant accumulation and the conservation audit between what was applied
//! and what the contacts carry.
//!
//! Literal port of `packages/analysis/unit-load/src/resultants.ts`.
//!
//! FLATTENING OF THE TAGGED SHAPES. The TypeScript reads and writes the app's
//! zod shapes, which tag every vector with a `kind` and a `unit`:
//!
//! * input `UnitLoadForceApplication` — `{applicationId, sourceKind, sourceId,
//!   point: {kind:"POINT",unit:"m",x,y,z}, force: {kind:"POLAR_VECTOR",
//!   unit:"N",x,y,z}, moment: {kind:"AXIAL_VECTOR",unit:"N_m",x,y,z}}`. The
//!   functions here read ONLY `point`, `force` and `moment`, so
//!   [`ForceApplication`] carries those three as plain [`Vec3`] and drops the
//!   identity fields and the unit tags. Callers (the ported `package-model` /
//!   `partition`) hold the full record and hand these three vectors over.
//! * input `PalletTopContactPatch` — the discriminated union of patch shapes;
//!   only `center`, `force` and `freeMoment` are read, so [`ContactPatch`]
//!   carries those three.
//! * output `Resultant` — `{force:{x:{unit:"N",value},y:…,z:…},
//!   moment:{x:{unit:"N_m",value},…}}`. Downstream reads `.force.x.value`,
//!   `.moment.y.value` and friends (see `package-model.ts:179-180`,
//!   `partition.ts:335-336`), so [`Resultant`] keeps the `force`/`moment`
//!   nesting and flattens `.x.value` to `.x`; the units are N and N_m by
//!   construction, exactly as the schema asserted.
//! * output `UnitLoadConservationAudit` — a flat record already; ported field
//!   for field as [`ConservationAudit`].
//! * the `tolerances` argument is structurally typed in TS (the caller passes
//!   the whole `NumericalAcceptanceProfile`, of which only two fields are
//!   read); here it is the two-field [`ResultantTolerances`].

use serde::{Deserialize, Serialize};

use crate::types::Vec3;

/// The numeric core of the app's `UnitLoadForceApplication`: where the force
/// acts, the force, and the free moment applied at that point.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ForceApplication {
    pub point: Vec3,
    pub force: Vec3,
    pub moment: Vec3,
}

/// The numeric core of the app's `PalletTopContactPatch`: the patch centre,
/// the force it transmits, and the couple it applies beyond that force.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ContactPatch {
    pub center: Vec3,
    pub force: Vec3,
    pub free_moment: Vec3,
}

/// A force/moment pair about the frame origin — the app's `Resultant` with the
/// per-component `{unit, value}` wrappers flattened away (see the module
/// essay).
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Resultant {
    pub force: Vec3,
    pub moment: Vec3,
}

/// The two fields `auditResultants` reads off the numerical acceptance
/// profile.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ResultantTolerances {
    pub force_tolerance_n: f64,
    pub moment_tolerance_nm: f64,
}

/// The app's `UnitLoadConservationAudit`.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ConservationAudit {
    pub expected: Resultant,
    pub observed: Resultant,
    pub force_residual_norm_n: f64,
    pub moment_residual_norm_nm: f64,
    pub accepted: bool,
}

/// TS `zero()`.
fn zero() -> Vec3 {
    Vec3 { x: 0.0, y: 0.0, z: 0.0 }
}

/// TS `add(left, right)` — component-wise, kept as its own function so the
/// nested `add(moment, add(cross(...), moment))` grouping below reads (and
/// rounds) exactly as the reference does.
fn add(left: Vec3, right: Vec3) -> Vec3 {
    Vec3 { x: left.x + right.x, y: left.y + right.y, z: left.z + right.z }
}

/// TS `cross(left, right)` — the same three expressions `Vec3::cross` carries.
fn cross(left: Vec3, right: Vec3) -> Vec3 {
    left.cross(right)
}

/// TS `resultant(force, moment)` — the schema parse is validation only and is
/// skipped per the porting rules; what survives is the shape it built.
fn resultant(force: Vec3, moment: Vec3) -> Resultant {
    Resultant { force, moment }
}

pub fn resultant_from_applications(applications: &[ForceApplication]) -> Resultant {
    let mut force = zero();
    let mut moment = zero();
    for application in applications {
        let applied_force = application.force;
        force = add(force, applied_force);
        moment = add(moment, add(cross(application.point, applied_force), application.moment));
    }
    resultant(force, moment)
}

pub fn resultant_from_contacts(contacts: &[ContactPatch]) -> Resultant {
    let mut force = zero();
    let mut moment = zero();
    for contact in contacts {
        force = add(force, contact.force);
        moment = add(moment, add(cross(contact.center, contact.force), contact.free_moment));
    }
    resultant(force, moment)
}

pub fn audit_resultants(
    expected: Resultant,
    observed: Resultant,
    tolerances: ResultantTolerances,
) -> ConservationAudit {
    // TS `Math.hypot(dx, dy, dz)`; `Vec3::hypot3` is the crate's agreed
    // rendering of the three-argument form.
    let force_residual_norm_n = Vec3 {
        x: expected.force.x - observed.force.x,
        y: expected.force.y - observed.force.y,
        z: expected.force.z - observed.force.z,
    }
    .hypot3();
    let moment_residual_norm_nm = Vec3 {
        x: expected.moment.x - observed.moment.x,
        y: expected.moment.y - observed.moment.y,
        z: expected.moment.z - observed.moment.z,
    }
    .hypot3();
    ConservationAudit {
        expected,
        observed,
        force_residual_norm_n,
        moment_residual_norm_nm,
        accepted: force_residual_norm_n <= tolerances.force_tolerance_n
            && moment_residual_norm_nm <= tolerances.moment_tolerance_nm,
    }
}

/// TS `export const vectorCross = cross;` — the same cross product, exported
/// under the name `package-model.ts` and `partition.ts` import it by.
pub fn vector_cross(left: Vec3, right: Vec3) -> Vec3 {
    cross(left, right)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn vector(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3 { x, y, z }
    }

    /// Three applications, hand-computed from the reference:
    ///
    /// 1. point (1,0,0), force (0,−10,0), moment (0,0,0)
    ///    → cross = (0·0−0·(−10), 0·0−1·0, 1·(−10)−0·0) = (0,0,−10)
    /// 2. point (0,0,2), force (3,0,0), moment (0,5,0)
    ///    → cross = (0·0−2·0, 2·3−0·0, 0·0−0·3) = (0,6,0); +moment = (0,11,0)
    /// 3. point (0.5,1,−2), force (0,−4,8), moment (1,−1,0.5)
    ///    → cross = (1·8−(−2)(−4), (−2)·0−0.5·8, 0.5·(−4)−1·0) = (0,−4,−2);
    ///      +moment = (1,−5,−1.5)
    ///
    /// force  = (0,−10,0)+(3,0,0)+(0,−4,8) = (3,−14,8)
    /// moment = (0,0,−10)+(0,11,0)+(1,−5,−1.5) = (1,6,−11.5)
    ///
    /// Every term is a dyadic rational, so the sums are exact in f64 and the
    /// assertion is exact equality.
    #[test]
    fn resultant_from_applications_accumulates_force_and_moment_levers() {
        let applications = [
            ForceApplication {
                point: vector(1.0, 0.0, 0.0),
                force: vector(0.0, -10.0, 0.0),
                moment: vector(0.0, 0.0, 0.0),
            },
            ForceApplication {
                point: vector(0.0, 0.0, 2.0),
                force: vector(3.0, 0.0, 0.0),
                moment: vector(0.0, 5.0, 0.0),
            },
            ForceApplication {
                point: vector(0.5, 1.0, -2.0),
                force: vector(0.0, -4.0, 8.0),
                moment: vector(1.0, -1.0, 0.5),
            },
        ];
        let total = resultant_from_applications(&applications);
        assert_eq!(total.force, vector(3.0, -14.0, 8.0));
        assert_eq!(total.moment, vector(1.0, 6.0, -11.5));
    }

    #[test]
    fn resultant_from_applications_of_nothing_is_zero() {
        let total = resultant_from_applications(&[]);
        assert_eq!(total.force, Vec3::ZERO);
        assert_eq!(total.moment, Vec3::ZERO);
    }

    /// Two patches, same arithmetic as the applications case with
    /// centre/freeMoment in place of point/moment:
    ///
    /// 1. centre (2,0,0), force (0,−100,0), free (0,0,0)
    ///    → cross = (0·0−0·(−100), 0·0−2·0, 2·(−100)−0·0) = (0,0,−200)
    /// 2. centre (−2,0,1), force (0,−100,0), free (0,0,25)
    ///    → cross = (0·0−1·(−100), 1·0−(−2)·0, (−2)(−100)−0·0) = (100,0,200);
    ///      +free = (100,0,225)
    ///
    /// force = (0,−200,0); moment = (100,0,25)
    #[test]
    fn resultant_from_contacts_uses_center_lever_and_free_moment() {
        let contacts = [
            ContactPatch {
                center: vector(2.0, 0.0, 0.0),
                force: vector(0.0, -100.0, 0.0),
                free_moment: vector(0.0, 0.0, 0.0),
            },
            ContactPatch {
                center: vector(-2.0, 0.0, 1.0),
                force: vector(0.0, -100.0, 0.0),
                free_moment: vector(0.0, 0.0, 25.0),
            },
        ];
        let total = resultant_from_contacts(&contacts);
        assert_eq!(total.force, vector(0.0, -200.0, 0.0));
        assert_eq!(total.moment, vector(100.0, 0.0, 25.0));
    }

    /// Force difference (3,4,12) → hypot = 13; moment difference (0,3,4) →
    /// hypot = 5. Both are exact Pythagorean chains, so equality is exact.
    /// Tolerances equal to the residuals accept, because the comparison is
    /// `<=` on both halves.
    #[test]
    fn audit_resultants_measures_residual_norms_and_accepts_on_the_tie() {
        let expected =
            Resultant { force: vector(3.0, 4.0, 12.0), moment: vector(0.0, 3.0, 4.0) };
        let observed = Resultant { force: Vec3::ZERO, moment: Vec3::ZERO };
        let audit = audit_resultants(
            expected,
            observed,
            ResultantTolerances { force_tolerance_n: 13.0, moment_tolerance_nm: 5.0 },
        );
        assert_eq!(audit.force_residual_norm_n, 13.0);
        assert_eq!(audit.moment_residual_norm_nm, 5.0);
        assert!(audit.accepted);
        assert_eq!(audit.expected, expected);
        assert_eq!(audit.observed, observed);
    }

    #[test]
    fn audit_resultants_rejects_when_either_half_exceeds_its_tolerance() {
        let expected =
            Resultant { force: vector(3.0, 4.0, 12.0), moment: vector(0.0, 3.0, 4.0) };
        let observed = Resultant { force: Vec3::ZERO, moment: Vec3::ZERO };
        let force_short = audit_resultants(
            expected,
            observed,
            ResultantTolerances { force_tolerance_n: 12.0, moment_tolerance_nm: 5.0 },
        );
        assert!(!force_short.accepted);
        let moment_short = audit_resultants(
            expected,
            observed,
            ResultantTolerances { force_tolerance_n: 13.0, moment_tolerance_nm: 4.0 },
        );
        assert!(!moment_short.accepted);
    }

    #[test]
    fn vector_cross_is_the_module_cross_product() {
        assert_eq!(
            vector_cross(vector(1.0, 0.0, 0.0), vector(0.0, 1.0, 0.0)),
            vector(0.0, 0.0, 1.0)
        );
        assert_eq!(
            vector_cross(vector(0.5, 1.0, -2.0), vector(0.0, -4.0, 8.0)),
            vector(0.0, -4.0, -2.0)
        );
    }
}
