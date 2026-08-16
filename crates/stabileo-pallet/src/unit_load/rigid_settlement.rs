//! THE RIGID-BODY SETTLEMENT SOLVE — one decomposition, two callers.
//!
//! A unit load standing on unilateral springs settles as a rigid body, and the
//! moment its resultant carries is transmitted the only way a bearing surface
//! can transmit one: differential vertical force. `advanceUnitLoadPartition`
//! has always used this solve to re-share the load each coupling round; the
//! INITIAL state (`compileContacts`) now uses the SAME solve with zero deck
//! response — the memoryless form's own first-round case — so the state the
//! pallet is first solved with and every state the advance produces obey ONE
//! decomposition. The initial state used to place authored uniform shares and
//! push the whole overturning couple into per-contact free moments instead:
//! measured at 0.4 g on a live stringer project, −320 N·m on every base
//! contact, applied by the projection as concentrated torques about single
//! deckboards' roll axes — a response the drift guard rightly refused, at
//! coupling round 0, before the first advance could ever heal the
//! disagreement.
//!
//! Literal port of `packages/analysis/unit-load/src/rigid-settlement.ts`. The
//! TS request is an inline object literal; here it is
//! [`RigidBodySettlementRequest`], and the `{reactionsN, settlementM}` return
//! is [`RigidBodySettlement`] — same field names, same order, no tagged
//! vectors anywhere in this file to flatten.

use serde::{Deserialize, Serialize};

use crate::types::{PalletError, PalletResult};

#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SettlementContact {
    pub stiffness_n_per_m: f64,
    pub deck_settlement_m: f64,
    /// How far the deck under this contact sinks per newton it is handed —
    /// MEASURED from the loop's own last two rounds, never assumed. Zero means
    /// "not measured yet", which is the honest reading of a first round and of
    /// a force that did not change enough to say anything.
    pub deck_compliance_m_per_n: f64,
    /// What this contact is carrying right now — the load the measured
    /// compliance has already sunk the deck by, and so the load to take back
    /// off it to find where the deck would be with this contact lifted.
    pub current_force_n: f64,
    pub x: f64,
    pub z: f64,
}

#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct RigidBodySettlementRequest {
    pub contacts: Vec<SettlementContact>,
    pub total_downward_force_n: f64,
    pub moment_x_target_nm: f64,
    pub moment_z_target_nm: f64,
}

#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct RigidBodySettlement {
    pub reactions_n: Vec<f64>,
    pub settlement_m: f64,
}

/// Solve `A·u = b` for a symmetric 3×3, or report that it cannot be solved.
///
/// Cramer's rule, because the system is three equations from three moments of
/// a contact set and the only interesting question about it is whether the set
/// is DEGENERATE — contacts on a line, or a single contact, which carry no
/// moment about that line and leave the tilt undetermined. A determinant
/// compared against the matrix's own scale answers that; a factorisation would
/// answer it the same way with more code.
fn solve_symmetric_3(a: &[f64; 9], b: &[f64; 3]) -> Option<[f64; 3]> {
    let [a00, a01, a02, a10, a11, a12, a20, a21, a22] = *a;
    let determinant = a00 * (a11 * a22 - a12 * a21) - a01 * (a10 * a22 - a12 * a20)
        + a02 * (a10 * a21 - a11 * a20);
    let scale = js_max_of(a.iter().map(|value| value.abs()));
    // TS `scale ** 3`, which is `Math.pow(scale, 3)` — `powf` is the same
    // library call; `powi`/`scale*scale*scale` is a different rounding.
    if scale == 0.0 || determinant.abs() <= 1e-12 * scale.powf(3.0) {
        return None;
    }
    let with_column = |index: usize| -> f64 {
        let mut copy = *a;
        copy[index] = b[0];
        copy[index + 3] = b[1];
        copy[index + 6] = b[2];
        let [c00, c01, c02, c10, c11, c12, c20, c21, c22] = copy;
        c00 * (c11 * c22 - c12 * c21) - c01 * (c10 * c22 - c12 * c20)
            + c02 * (c10 * c21 - c11 * c20)
    };
    Some([
        with_column(0) / determinant,
        with_column(1) / determinant,
        with_column(2) / determinant,
    ])
}

/// The nine running sums the moment system is assembled from — TS's reduce
/// accumulator object, rebuilt per contact in the same order so the additions
/// round identically.
#[derive(Debug, Clone, Copy, Default)]
struct MomentSums {
    k: f64,
    kx: f64,
    kz: f64,
    kxx: f64,
    kxz: f64,
    kzz: f64,
    kg: f64,
    kgx: f64,
    kgz: f64,
}

/// `Math.max(...values)` with JS semantics: NaN is contagious, where Rust's
/// `f64::max` swallows it; the empty spread is `-Infinity`, which is what the
/// seed below is. The matrix always has nine entries, so only the NaN half is
/// observable.
fn js_max_of(values: impl Iterator<Item = f64>) -> f64 {
    let mut result = f64::NEG_INFINITY;
    for value in values {
        if value.is_nan() {
            return f64::NAN;
        }
        if value > result {
            result = value;
        }
    }
    result
}

/// `Math.max(left, right)` with JS semantics (see [`js_max_of`]).
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

/// Σ k(w − g) = W over the active set, with the tilt held at zero.
fn level_settlement(
    contacts: &[SettlementContact],
    active: &[bool],
    total_downward_force_n: f64,
) -> f64 {
    let mut stiffness = 0.0;
    for (index, contact) in contacts.iter().enumerate() {
        if active[index] {
            stiffness += contact.stiffness_n_per_m;
        }
    }
    if stiffness <= 0.0 {
        return 0.0;
    }
    let mut weighted = 0.0;
    for (index, contact) in contacts.iter().enumerate() {
        if active[index] {
            weighted += contact.stiffness_n_per_m * contact.deck_settlement_m;
        }
    }
    (total_downward_force_n + weighted) / stiffness
}

/// THE LOAD BODY'S OWN SETTLEMENT, and which contacts it leaves behind.
///
/// `w(r) = w₀ + wₓ·x + w_z·z` is the rigid body's descent and tilt; each
/// contact carries `k·(w(r) − deck settlement)` and nothing may PULL. The
/// active set is the classic one: solve, drop whatever came out negative,
/// re-solve, and then re-admit anything the new settlement has pressed back
/// into the deck. It terminates because a contact dropped for pulling is never
/// re-admitted in the same pass unless the settlement moved onto it, and the
/// pass count is bounded by the contact count either way — an exhausted budget
/// throws rather than returning a state nobody solved.
///
/// A DEGENERATE SET IS NOT AN ERROR. One contact, or contacts on a line, carry
/// no moment about that line, so the tilt is undetermined rather than wrong;
/// the solve falls back to a level settlement, which is exactly what a load on
/// a single row of contacts does. The moment it cannot carry is left to the
/// residual the caller already distributes.
// The TS seeds `reactions` with a zero per contact before the pass loop; the
// loop always reassigns it before anything reads it, so Rust calls the seed
// dead. It is kept because the reference has it — the shape of the walk is the
// thing being ported.
#[allow(unused_assignments)]
pub fn solve_rigid_body_settlement(
    request: &RigidBodySettlementRequest,
) -> PalletResult<RigidBodySettlement> {
    let total_downward_force_n = request.total_downward_force_n;
    // THE CONTACT AND THE DECK UNDER IT ARE TWO SPRINGS IN SERIES, and
    // pretending the deck is a fixed shape is what made this walk a bang-bang
    // one.
    //
    // The load stands on its own interface spring, and that spring stands on a
    // deck which sinks under whatever it is handed. On the born pallet the
    // interface is about twenty-five times the stiffer of the two, so a tenth
    // of a millimetre of deck movement swings a contact between carrying a
    // kilonewton and carrying nothing — and an iteration that recomputes the
    // forces against a deck shape held FIXED swings with it. Combined in
    // series, the pair is as soft as the softer of them, and the swing goes
    // with it.
    //
    //   k_series = k / (1 + k·c)      the two springs, end to end
    //   g_unloaded = g − c·F          where the deck would be with this
    //                                 contact off
    //
    // THE FIXED POINT IS UNTOUCHED. Substituting `g = g_unloaded + c·F` back
    // into `F = k_series·(w − g_unloaded)` returns `F = k·(w − g)` exactly:
    // the state where the load, the springs and the deck all agree is the same
    // state it always was. What changes is only how the walk gets there —
    // numerics, not physics, and the compliance is measured rather than
    // chosen, so no constant enters the product.
    let contacts: Vec<SettlementContact> = request
        .contacts
        .iter()
        .map(|contact| {
            let compliance = contact.deck_compliance_m_per_n;
            // TS `if (!(compliance > 0)) return contact;` — the negated form,
            // so a NaN compliance keeps the contact untouched.
            if !(compliance > 0.0) {
                return *contact;
            }
            let series =
                contact.stiffness_n_per_m / (1.0 + contact.stiffness_n_per_m * compliance);
            SettlementContact {
                stiffness_n_per_m: series,
                deck_settlement_m: contact.deck_settlement_m
                    - compliance * contact.current_force_n,
                ..*contact
            }
        })
        .collect();
    let mut active: Vec<bool> = contacts.iter().map(|_| true).collect();
    let mut reactions: Vec<f64> = contacts.iter().map(|_| 0.0).collect();
    for _pass in 0..=(contacts.len() + 1) {
        let mut moments = MomentSums::default();
        for (index, contact) in contacts.iter().enumerate() {
            if !active[index] {
                continue;
            }
            let k = contact.stiffness_n_per_m;
            moments = MomentSums {
                k: moments.k + k,
                kx: moments.kx + k * contact.x,
                kz: moments.kz + k * contact.z,
                kxx: moments.kxx + k * contact.x * contact.x,
                kxz: moments.kxz + k * contact.x * contact.z,
                kzz: moments.kzz + k * contact.z * contact.z,
                kg: moments.kg + k * contact.deck_settlement_m,
                kgx: moments.kgx + k * contact.deck_settlement_m * contact.x,
                kgz: moments.kgz + k * contact.deck_settlement_m * contact.z,
            };
        }
        // Σ F = W ; Σ F·x = −Mz ; Σ F·z = Mx — the vertical half of the load's
        // own resultant, in the frame's own sign convention.
        let solved = solve_symmetric_3(
            &[
                moments.k,
                moments.kx,
                moments.kz,
                moments.kx,
                moments.kxx,
                moments.kxz,
                moments.kz,
                moments.kxz,
                moments.kzz,
            ],
            &[
                total_downward_force_n + moments.kg,
                -request.moment_z_target_nm + moments.kgx,
                request.moment_x_target_nm + moments.kgz,
            ],
        );
        let level = if solved.is_none() {
            level_settlement(&contacts, &active, total_downward_force_n)
        } else {
            0.0
        };
        let settlement_at = |contact: &SettlementContact| -> f64 {
            match solved {
                None => level,
                Some(solved) => solved[0] + solved[1] * contact.x + solved[2] * contact.z,
            }
        };
        reactions = contacts
            .iter()
            .enumerate()
            .map(|(index, contact)| {
                if active[index] {
                    contact.stiffness_n_per_m * (settlement_at(contact) - contact.deck_settlement_m)
                } else {
                    0.0
                }
            })
            .collect();

        // Nothing may pull: drop the worst offender and solve the smaller
        // problem.
        let mut worst_pulling: isize = -1;
        for (index, reaction) in reactions.iter().enumerate() {
            if !active[index] || *reaction >= 0.0 {
                continue;
            }
            if worst_pulling < 0 || *reaction < reactions[worst_pulling as usize] {
                worst_pulling = index as isize;
            }
        }
        if worst_pulling >= 0 {
            active[worst_pulling as usize] = false;
            continue;
        }
        // ...and nothing may hover while the deck presses into it.
        let mut deepest: isize = -1;
        for (index, contact) in contacts.iter().enumerate() {
            if active[index] {
                continue;
            }
            let penetration = settlement_at(contact) - contact.deck_settlement_m;
            if penetration <= 0.0 {
                continue;
            }
            if deepest < 0
                || penetration
                    > settlement_at(&contacts[deepest as usize])
                        - contacts[deepest as usize].deck_settlement_m
            {
                deepest = index as isize;
            }
        }
        if deepest >= 0 {
            active[deepest as usize] = true;
            continue;
        }
        return Ok(RigidBodySettlement {
            reactions_n: reactions.iter().map(|reaction| js_max(0.0, *reaction)).collect(),
            settlement_m: match solved {
                Some(solved) => solved[0],
                None => level,
            },
        });
    }
    Err(PalletError::sentence("CONTACT_ACTIVE_SET_DID_NOT_SETTLE"))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn contact(x: f64, z: f64, stiffness_n_per_m: f64) -> SettlementContact {
        SettlementContact {
            stiffness_n_per_m,
            deck_settlement_m: 0.0,
            deck_compliance_m_per_n: 0.0,
            current_force_n: 0.0,
            x,
            z,
        }
    }

    /// A square of four equal springs at (±1, ±1), k = 1000 N/m each, on a
    /// flat deck, carrying 4000 N with no moment.
    ///
    /// The assembled system is diagonal: k = 4000, kxx = kzz = 4000, kx = kz =
    /// kxz = 0, and every kg term is zero. So `A·u = b` is
    /// `diag(4000)·u = (4000, 0, 0)` → w = (1, 0, 0): the body sinks one metre
    /// level, and each contact carries 1000·(1 − 0) = 1000 N. Every quantity
    /// is exact in f64 (the Cramer quotients are 6.4e10 ratios of exact
    /// integers), so the assertions are exact.
    #[test]
    fn a_symmetric_square_shares_the_load_evenly() {
        let settlement = solve_rigid_body_settlement(&RigidBodySettlementRequest {
            contacts: vec![
                contact(1.0, 1.0, 1000.0),
                contact(1.0, -1.0, 1000.0),
                contact(-1.0, 1.0, 1000.0),
                contact(-1.0, -1.0, 1000.0),
            ],
            total_downward_force_n: 4000.0,
            moment_x_target_nm: 0.0,
            moment_z_target_nm: 0.0,
        })
        .expect("settles");
        assert_eq!(settlement.reactions_n, vec![1000.0, 1000.0, 1000.0, 1000.0]);
        assert_eq!(settlement.settlement_m, 1.0);
    }

    /// The same square with an overturning couple about z: Mz = −2000 N·m puts
    /// b = (4000, 2000, 0) → w = (1, 0.5, 0), i.e. a half-radian-per-metre
    /// tilt in x. Reactions are 1000·(1 ± 0.5): 1500 on the x = +1 pair, 500
    /// on the x = −1 pair. Σ F = 4000 ✓ and Σ F·x = 1500 + 1500 − 500 − 500 =
    /// 2000 = −Mz ✓ — the sign convention the assembly comment states.
    #[test]
    fn a_moment_tilts_the_body_and_differentiates_the_shares() {
        let settlement = solve_rigid_body_settlement(&RigidBodySettlementRequest {
            contacts: vec![
                contact(1.0, 1.0, 1000.0),
                contact(1.0, -1.0, 1000.0),
                contact(-1.0, 1.0, 1000.0),
                contact(-1.0, -1.0, 1000.0),
            ],
            total_downward_force_n: 4000.0,
            moment_x_target_nm: 0.0,
            moment_z_target_nm: -2000.0,
        })
        .expect("settles");
        assert_eq!(settlement.reactions_n, vec![1500.0, 1500.0, 500.0, 500.0]);
        assert_eq!(settlement.settlement_m, 1.0);
    }

    /// THE DROP AND RE-SOLVE. Same square, W = 4500 N, Mx = 2500 N·m,
    /// Mz = −2500 N·m.
    ///
    /// Pass 0 — all four active, the diagonal system again:
    ///   w = (4500/4000, 2500/4000, 2500/4000) = (1.125, 0.625, 0.625)
    ///   reactions = 1000·(1.125 ± 0.625 ± 0.625)
    ///             = [2375, 1125, 1125, −125]
    /// The last one PULLS, so it is dropped and the pass restarts.
    ///
    /// Pass 1 — active {(1,1), (1,−1), (−1,1)}:
    ///   k = 3000, kx = kz = 1000, kxx = kzz = 3000, kxz = −1000
    ///   A = 1000·[[3,1,1],[1,3,−1],[1,−1,3]], b = (4500, 2500, 2500)
    ///   det(A/1000) = 3(9−1) − 1(3+1) + 1(−1−3) = 16
    ///   numerators = 8, 12, 12 (×1000 each) → w = (1, 0.75, 0.75)
    ///   reactions = 1000·(1 + 0.75x + 0.75z) = [2500, 1000, 1000, 0]
    /// Nothing pulls, and the dropped contact's penetration is
    /// 1 − 0.75 − 0.75 = −0.5 ≤ 0, so it stays out and the pass returns.
    ///
    /// Σ F = 4500 ✓, Σ F·x = 2500 + 1000 − 1000 = 2500 = −Mz ✓,
    /// Σ F·z = 2500 − 1000 + 1000 = 2500 = Mx ✓. All the arithmetic is on
    /// exact integers and dyadic rationals, so the assertions are exact.
    #[test]
    fn a_pulling_contact_is_dropped_and_the_rest_re_solve() {
        let settlement = solve_rigid_body_settlement(&RigidBodySettlementRequest {
            contacts: vec![
                contact(1.0, 1.0, 1000.0),
                contact(1.0, -1.0, 1000.0),
                contact(-1.0, 1.0, 1000.0),
                contact(-1.0, -1.0, 1000.0),
            ],
            total_downward_force_n: 4500.0,
            moment_x_target_nm: 2500.0,
            moment_z_target_nm: -2500.0,
        })
        .expect("settles");
        assert_eq!(settlement.reactions_n, vec![2500.0, 1000.0, 1000.0, 0.0]);
        assert_eq!(settlement.settlement_m, 1.0);
    }

    /// A DEGENERATE SET IS NOT AN ERROR: two contacts carry no moment about
    /// the line through them, so the 3×3 is singular (rows 0 and 1 of the
    /// assembled matrix are identical here) and the solve falls back to the
    /// level settlement — (4000 + 0)/4000 = 1 — which the stiffer contact
    /// turns into the larger share: 1000·1 and 3000·1.
    #[test]
    fn a_degenerate_set_falls_back_to_a_level_settlement() {
        let settlement = solve_rigid_body_settlement(&RigidBodySettlementRequest {
            contacts: vec![contact(1.0, 1.0, 1000.0), contact(1.0, -1.0, 3000.0)],
            total_downward_force_n: 4000.0,
            moment_x_target_nm: 0.0,
            moment_z_target_nm: 0.0,
        })
        .expect("settles");
        assert_eq!(settlement.reactions_n, vec![1000.0, 3000.0]);
        assert_eq!(settlement.settlement_m, 1.0);
    }

    /// THE SERIES SPRING. Two contacts with k = 1000 N/m each on a deck of
    /// measured compliance 0.001 m/N, currently carrying 100 N apiece:
    ///
    ///   k_series   = 1000/(1 + 1000·0.001) = 500 N/m
    ///   g_unloaded = 0 − 0.001·100 = −0.1 m
    ///
    /// The pair is collinear, so the level branch runs:
    ///   level = (4000 + (500·(−0.1) + 500·(−0.1)))/1000 = 3900/1000 = 3.9 m
    ///   reaction = 500·(3.9 − (−0.1)) = 500·4 = 2000 N each
    ///
    /// The fixed point is untouched: 2000 N at series stiffness is the same
    /// state as `k·(w − g)` on the original spring, which is what the essay
    /// above claims and what the totals here confirm (Σ = 4000 = W).
    #[test]
    fn a_measured_deck_compliance_softens_the_contact_in_series() {
        let compliant = SettlementContact {
            stiffness_n_per_m: 1000.0,
            deck_settlement_m: 0.0,
            deck_compliance_m_per_n: 0.001,
            current_force_n: 100.0,
            x: 1.0,
            z: 0.0,
        };
        let settlement = solve_rigid_body_settlement(&RigidBodySettlementRequest {
            contacts: vec![
                SettlementContact { z: 1.0, ..compliant },
                SettlementContact { z: -1.0, ..compliant },
            ],
            total_downward_force_n: 4000.0,
            moment_x_target_nm: 0.0,
            moment_z_target_nm: 0.0,
        })
        .expect("settles");
        assert_eq!(settlement.reactions_n, vec![2000.0, 2000.0]);
        assert_eq!(settlement.settlement_m, 3.9);
    }

    /// A load with no contacts has nothing to settle on: the system is all
    /// zeros, `scale === 0` short-circuits the solve, and the level fallback
    /// returns 0 for a zero total stiffness. No reactions, no settlement, no
    /// throw.
    #[test]
    fn no_contacts_settle_nowhere() {
        let settlement = solve_rigid_body_settlement(&RigidBodySettlementRequest {
            contacts: vec![],
            total_downward_force_n: 4000.0,
            moment_x_target_nm: 0.0,
            moment_z_target_nm: 0.0,
        })
        .expect("settles");
        assert_eq!(settlement.reactions_n, Vec::<f64>::new());
        assert_eq!(settlement.settlement_m, 0.0);
    }

    /// THE BUDGET IS NOT A FORMALITY. The same square under a couple too large
    /// for it — W = 4000 N, Mz = −6000 N·m — cycles: pass 0 drops (−1,1) [the
    /// first of the two equally-worst pullers], pass 1 drops (−1,−1), pass 2
    /// is the degenerate x = 1 pair whose level settlement presses the deck
    /// into (−1,1) and re-admits it, pass 3 drops it again, and so on. The
    /// loop runs `contacts.length + 2` passes and then refuses to return a
    /// state nobody solved.
    #[test]
    fn an_active_set_that_cannot_settle_is_refused() {
        let error = solve_rigid_body_settlement(&RigidBodySettlementRequest {
            contacts: vec![
                contact(1.0, 1.0, 1000.0),
                contact(1.0, -1.0, 1000.0),
                contact(-1.0, 1.0, 1000.0),
                contact(-1.0, -1.0, 1000.0),
            ],
            total_downward_force_n: 4000.0,
            moment_x_target_nm: 0.0,
            moment_z_target_nm: -6000.0,
        })
        .expect_err("refused");
        assert_eq!(error.code, "CONTACT_ACTIVE_SET_DID_NOT_SETTLE");
        assert_eq!(error.message, "CONTACT_ACTIVE_SET_DID_NOT_SETTLE");
    }
}
