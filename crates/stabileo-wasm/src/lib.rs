//! The one shipped WASM binary. Linking the kernel crate re-exports its
//! entire `#[wasm_bindgen]` surface unchanged — every existing operation
//! keeps its name and signature — and the pallet orchestration adds its own
//! entries beside them.

pub use dedaliano_engine::*;
