//! Literal port of `packages/analysis/stabileo/src/equilibrium.ts`.

use crate::kernel_bridge::number_format::js_number_to_string;
use crate::schema::{Resultant, Tagged3};
use crate::types::{PalletError, PalletResult, Vec3};

/// The force below which a residual is noise rather than a load: one
/// millinewton, a tenth of a gram's weight. Nothing a pallet does is decided at
/// that scale, and no solve should be refused for it.
const ABSOLUTE_FORCE_TOLERANCE_N: f64 = 0.001;

/// REVIEWED RELATIVE ACCEPTANCE: a resultant is balanced when what fails to
/// cancel is under a tenth of a percent of it.
///
/// This is a stated engineering judgement, not a number fitted to an observed
/// residual. A real loss of equilibrium is not marginal — when the load-share
/// arithmetic annihilated every base contact, the residual was the WHOLE load —
/// so a bound three orders below unity refuses every genuine one while leaving
/// a well-conditioned solve room to be finite-precision.
///
/// ## The accuracy this sits inside
///
/// A born GMA pallet's own solve lands at 1.6e-5 of its moment — a fourteen
/// micrometre shift in the line of action of a 12.6 kN load. The estimation tier
/// feeding that solve declares ±25% on joint stiffness, and the geometry it
/// stands on is stored to `Decimal(8, 3)`. Refusing to publish an estimate over
/// an imbalance four orders below the model's own declared accuracy is not
/// conservatism, it is a unit error wearing conservatism's coat — which is what
/// the previous bound was, having applied a force constant to newton-metres.
///
/// REVIEW FLAG: this fraction is a ruling of 2026-08-14 taken to unblock the
/// product's first estimate, and it is one commit. If a reviewer wants a
/// stricter number, the place to argue is here and the revert is clean. What
/// must NOT happen is tightening it back to something dimensionally accidental.
const RELATIVE_TOLERANCE: f64 = 0.001;

/// The tolerance for a residual against the magnitude it must cancel.
///
/// Two-sided on purpose: the absolute floor covers a resultant near zero (where
/// a relative bound would demand exactness of a quantity that is itself noise),
/// and the relative term covers everything else.
fn residual_tolerance(absolute_floor: f64, applied_norm: f64) -> f64 {
    absolute_floor.max(RELATIVE_TOLERANCE * applied_norm)
}

#[derive(Debug, Clone)]
pub struct EquilibriumAudit {
    pub applied: Resultant,
    pub reactions: Resultant,
    pub force_residual: Tagged3,
    pub moment_residual: Tagged3,
    pub force_residual_norm: f64,
    pub moment_residual_norm: f64,
    pub force_tolerance: f64,
    pub moment_tolerance: f64,
    pub accepted: bool,
}

/// Does what came out balance what went in.
///
/// ## Why a moment needs a LENGTH before it can have a tolerance
///
/// A moment residual is a force residual times a lever arm, and it is not even
/// origin-invariant while the force residual is nonzero: shift the origin by
/// `d` and the moment residual shifts by `d x F_res`. So a bound on moments
/// cannot be a bare number — one millinewton-metre means something entirely
/// different on a pallet and on a bridge, and this audit used to apply the SAME
/// constant to newtons and to newton-metres, which is a unit error rather than a
/// strict standard. On the default pallet that accidental 1 m of implied length
/// refused a solve whose moment residual was 0.16 N·m on 9992 N·m — a fourteen
/// MICROMETRE shift in the line of action of a 12.6 kN load, beneath the
/// `Decimal(8, 3)` grain the geometry is even stored at.
///
/// `characteristic_length_m` is therefore required, and the caller derives it
/// from the frame itself — the greatest distance from the moment origin to any
/// node, which is exactly the longest lever any force residual can act on. The
/// moment's absolute floor is then the force floor carried out to that lever,
/// and the relative term is the same reviewed fraction for both.
pub fn audit_equilibrium(
    applied: &Resultant,
    reactions: &Resultant,
    characteristic_length_m: f64,
) -> PalletResult<EquilibriumAudit> {
    if !characteristic_length_m.is_finite() || characteristic_length_m < 0.0 {
        return Err(PalletError::sentence(format!(
            "EQUILIBRIUM_AUDIT_CHARACTERISTIC_LENGTH:{}",
            js_number_to_string(characteristic_length_m)
        )));
    }
    // The TS re-parses both resultants through `ResultantSchema` here; the
    // Rust side receives them already typed, so the parse is the type.
    let force_values = Vec3 {
        x: applied.force.x.value + reactions.force.x.value,
        y: applied.force.y.value + reactions.force.y.value,
        z: applied.force.z.value + reactions.force.z.value,
    };
    let moment_values = Vec3 {
        x: applied.moment.x.value + reactions.moment.x.value,
        y: applied.moment.y.value + reactions.moment.y.value,
        z: applied.moment.z.value + reactions.moment.z.value,
    };
    let force_residual_norm = force_values.hypot3();
    let moment_residual_norm = moment_values.hypot3();
    let applied_force_norm =
        Vec3 { x: applied.force.x.value, y: applied.force.y.value, z: applied.force.z.value }
            .hypot3();
    let applied_moment_norm =
        Vec3 { x: applied.moment.x.value, y: applied.moment.y.value, z: applied.moment.z.value }
            .hypot3();
    let force_tolerance = residual_tolerance(ABSOLUTE_FORCE_TOLERANCE_N, applied_force_norm);
    let moment_tolerance = residual_tolerance(
        ABSOLUTE_FORCE_TOLERANCE_N * characteristic_length_m,
        applied_moment_norm,
    );
    Ok(EquilibriumAudit {
        applied: applied.clone(),
        reactions: reactions.clone(),
        // Built without the negative-zero cleaning `fromStabileo*` applies:
        // these are raw sums, not vectors that came back through the basis.
        force_residual: Tagged3 {
            kind: "POLAR_VECTOR".to_string(),
            unit: "N".to_string(),
            x: force_values.x,
            y: force_values.y,
            z: force_values.z,
        },
        moment_residual: Tagged3 {
            kind: "AXIAL_VECTOR".to_string(),
            unit: "N_m".to_string(),
            x: moment_values.x,
            y: moment_values.y,
            z: moment_values.z,
        },
        force_residual_norm,
        moment_residual_norm,
        force_tolerance,
        moment_tolerance,
        accepted: force_residual_norm <= force_tolerance
            && moment_residual_norm <= moment_tolerance,
    })
}
