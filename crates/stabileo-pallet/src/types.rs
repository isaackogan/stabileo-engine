//! Shared numeric primitives for the pallet orchestration. App-schema shapes
//! (frames, states) live beside the logic that owns them; only the vocabulary
//! every module speaks lives here.

use serde::{Deserialize, Serialize};

/// A plain 3-vector. App payloads wrap vectors with `kind`/`unit` tags; the
/// tagged forms deserialize into their own structs and convert to this for
/// arithmetic.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const ZERO: Vec3 = Vec3 { x: 0.0, y: 0.0, z: 0.0 };

    pub fn cross(self, other: Vec3) -> Vec3 {
        Vec3 {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }

    pub fn hypot3(self) -> f64 {
        // TS Math.hypot(x, y, z): sqrt of the sum of squares, computed the
        // same way (f64 hypot chains differ in the last ulps from sqrt of
        // sums; the reference uses Math.hypot, which is IEEE-correct — Rust's
        // two-arg f64::hypot chained matches it closely, but the gate
        // compares against a tolerance, not bit equality, for these).
        self.x.hypot(self.y).hypot(self.z)
    }
}

/// A planar (x, z) pair — tangential quantities on the floor plane.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct PlanarXZ {
    pub x: f64,
    pub z: f64,
}

/// The error carrier for the whole crate: a stable CODE plus the exact
/// sentence the TypeScript reference threw. Consumers pattern-match the
/// strings, so both halves are contract.
#[derive(Debug, Clone)]
pub struct PalletError {
    pub code: String,
    pub message: String,
}

impl PalletError {
    pub fn new(code: impl Into<String>, message: impl Into<String>) -> Self {
        PalletError { code: code.into(), message: message.into() }
    }

    /// The TS idiom `throw new TypeError("CODE:detail")` where code and
    /// message share one string.
    pub fn sentence(sentence: impl Into<String>) -> Self {
        let sentence = sentence.into();
        let code = sentence.split(':').next().unwrap_or("").to_string();
        PalletError { code, message: sentence }
    }
}

impl std::fmt::Display for PalletError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(formatter, "{}", self.message)
    }
}

impl std::error::Error for PalletError {}

pub type PalletResult<T> = Result<T, PalletError>;
