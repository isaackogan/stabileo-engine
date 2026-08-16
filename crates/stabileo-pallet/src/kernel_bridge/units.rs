//! Literal port of `packages/analysis/stabileo/src/units.ts`.
//!
//! The app speaks SI (newtons, newton-metres, pascals, newtons per metre); the
//! kernel speaks kilonewtons and megapascals. Every crossing is one of these
//! functions and nothing else.

use crate::types::{PalletError, PalletResult};

fn finite(value: f64) -> PalletResult<f64> {
    if !value.is_finite() {
        return Err(PalletError::sentence("Stabileo value must be finite"));
    }
    Ok(value)
}

pub fn solver_length_to_sdk(metres: f64) -> PalletResult<f64> {
    finite(metres)
}

pub fn solver_force_value_to_sdk(newtons: f64) -> PalletResult<f64> {
    Ok(finite(newtons)? / 1_000.0)
}

pub fn solver_moment_value_to_sdk(newton_metres: f64) -> PalletResult<f64> {
    Ok(finite(newton_metres)? / 1_000.0)
}

pub fn solver_modulus_value_to_sdk(pascals: f64) -> PalletResult<f64> {
    Ok(finite(pascals)? / 1_000_000.0)
}

/// A STIFFNESS is a force per length, so it converts the way a force does and
/// the length cancels: N/m -> kN/m.
///
/// Worth its own function rather than a call to `solver_force_value_to_sdk`:
/// passing a floor's 1.0e9 N/m through unconverted makes it a 1.0e12 N/m floor,
/// which is still "rigid" and therefore still produces a plausible-looking
/// solve. A thousandfold error that hides is the kind worth naming.
pub fn solver_stiffness_value_to_sdk(newtons_per_metre: f64) -> PalletResult<f64> {
    Ok(finite(newtons_per_metre)? / 1_000.0)
}

pub fn sdk_force_value_to_solver(kilonewtons: f64) -> PalletResult<f64> {
    Ok(finite(kilonewtons)? * 1_000.0)
}

pub fn sdk_moment_value_to_solver(kilonewton_metres: f64) -> PalletResult<f64> {
    Ok(finite(kilonewton_metres)? * 1_000.0)
}
