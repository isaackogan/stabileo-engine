//! The unit-load leaf mechanics, ported from
//! `packages/analysis/unit-load/src/*` in the consuming application.
//!
//! Each module is a literal translation of one TypeScript file: the resultant
//! accumulation and its conservation audit, the Coulomb two-tangent rule, the
//! capacity criterion evaluator, the unilateral normal-contact rule, and the
//! rigid-body settlement solve with its active set.

pub mod contact;
pub mod criteria;
pub mod friction;
pub mod resultants;
pub mod rigid_settlement;
