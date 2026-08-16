//! Coupled pallet/unit-load analysis on the Dedaliano kernel.
//!
//! This crate is the engine-side home of the orchestration that used to live
//! in the consuming application: the unilateral floor contact search, the
//! Coulomb stick/slip rules, the rigid-body load partition with measured deck
//! compliance, and the coupled fixed-point loop that ties them together. One
//! call solves one load event; the kernel is invoked natively per inner
//! round, with no serialization between rounds.

pub mod schema;
