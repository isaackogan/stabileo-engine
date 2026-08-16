//! Literal port of `packages/analysis/stabileo/src/coordinates.ts`.
//!
//! The frame's basis and the kernel's basis differ by one rotation, and every
//! vector that crosses the boundary goes through it. The app's tagged vector
//! kinds (POINT / POLAR_VECTOR / AXIAL_VECTOR) travel the SAME rotation — the
//! distinction is carried for the schema, not for the arithmetic.

use crate::schema::Tagged3;
use crate::types::{PalletError, PalletResult, Vec3};

/// The frame's up-axis is the SDK's z, and the handedness has to survive:
/// (x, y, z) -> (x, -z, y).
fn rotate(vector: Vec3) -> Vec3 {
    Vec3 { x: vector.x, y: -vector.z, z: vector.y }
}

/// TS `Object.is(value, -0) ? 0 : value` — negative zero is normalized away on
/// the way back so a serialized payload never carries `-0`.
fn clean_zero(value: f64) -> f64 {
    if value == 0.0 {
        0.0
    } else {
        value
    }
}

fn inverse_rotate(vector: Vec3) -> Vec3 {
    Vec3 { x: clean_zero(vector.x), y: clean_zero(vector.z), z: clean_zero(-vector.y) }
}

pub fn to_stabileo_point(point: Vec3) -> Vec3 {
    rotate(point)
}

pub fn to_stabileo_polar(vector: Vec3) -> Vec3 {
    rotate(vector)
}

pub fn to_stabileo_axial(vector: Vec3) -> Vec3 {
    rotate(vector)
}

pub fn from_stabileo_point(point: Vec3) -> Tagged3 {
    Tagged3::point_m(inverse_rotate(point))
}

pub fn from_stabileo_polar(vector: Vec3, unit: &str) -> Tagged3 {
    Tagged3::polar(unit, inverse_rotate(vector))
}

pub fn from_stabileo_axial(vector: Vec3, unit: &str) -> Tagged3 {
    Tagged3::axial(unit, inverse_rotate(vector))
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StabileoLocalTriad {
    pub x: Vec3,
    pub y: Vec3,
    pub z: Vec3,
}

fn dot(left: Vec3, right: Vec3) -> f64 {
    left.x * right.x + left.y * right.y + left.z * right.z
}

fn cross(left: Vec3, right: Vec3) -> Vec3 {
    Vec3 {
        x: left.y * right.z - left.z * right.y,
        y: left.z * right.x - left.x * right.z,
        z: left.x * right.y - left.y * right.x,
    }
}

fn scale(vector: Vec3, factor: f64) -> Vec3 {
    Vec3 { x: vector.x * factor, y: vector.y * factor, z: vector.z * factor }
}

fn subtract(left: Vec3, right: Vec3) -> Vec3 {
    Vec3 { x: left.x - right.x, y: left.y - right.y, z: left.z - right.z }
}

fn normalize(vector: Vec3, label: &str) -> PalletResult<Vec3> {
    if !(vector.x.is_finite() && vector.y.is_finite() && vector.z.is_finite()) {
        return Err(PalletError::sentence(format!("{label} contains a non-finite component")));
    }
    let length = vector.hypot3();
    if length <= 1e-12 {
        return Err(PalletError::sentence(format!("Degenerate local axis: {label}")));
    }
    Ok(scale(vector, 1.0 / length))
}

pub fn to_stabileo_local_triad(x: Vec3, local_y: Vec3) -> PalletResult<StabileoLocalTriad> {
    let frame_x = normalize(x, "member x axis")?;
    let projected_y = subtract(local_y, scale(frame_x, dot(local_y, frame_x)));
    let frame_y = normalize(projected_y, "local y axis")?;
    let frame_z = normalize(cross(frame_x, frame_y), "local z axis")?;
    Ok(StabileoLocalTriad { x: rotate(frame_x), y: rotate(frame_y), z: rotate(frame_z) })
}
