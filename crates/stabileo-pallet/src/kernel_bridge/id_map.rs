//! Literal port of `packages/analysis/stabileo/src/id-map.ts`.
//!
//! The kernel numbers everything; the app names everything. The map between
//! the two is derived from the stable ids alone — sorted bytewise, numbered
//! from one — so the same frame always compiles to the same numeric model.

use std::cmp::Ordering;

use crate::types::{PalletError, PalletResult};

/// TS `bytewiseUtf8Compare`: compare UTF-8 bytes, then length. Rust's own
/// `str` ordering is byte-lexicographic and would agree, but the reference
/// sorts through this function and the port keeps the same call.
pub fn bytewise_utf8_compare(left: &str, right: &str) -> Ordering {
    let left_bytes = left.as_bytes();
    let right_bytes = right.as_bytes();
    let length = left_bytes.len().min(right_bytes.len());
    for index in 0..length {
        let difference = left_bytes[index] as i32 - right_bytes[index] as i32;
        if difference != 0 {
            return difference.cmp(&0);
        }
    }
    left_bytes.len().cmp(&right_bytes.len())
}

#[derive(Debug, Clone)]
pub struct NumericIdMap {
    entries: Vec<(String, usize)>,
}

impl NumericIdMap {
    pub fn build(ids: &[String]) -> PalletResult<NumericIdMap> {
        let mut sorted: Vec<String> = ids.to_vec();
        sorted.sort_by(|left, right| bytewise_utf8_compare(left, right));
        for index in 1..sorted.len() {
            if sorted[index - 1] == sorted[index] {
                return Err(PalletError::sentence(format!("DUPLICATE_STABLE_ID: {}", sorted[index])));
            }
        }
        let entries = sorted
            .into_iter()
            .enumerate()
            .map(|(index, stable_id)| (stable_id, index + 1))
            .collect();
        Ok(NumericIdMap { entries })
    }

    pub fn entries(&self) -> &[(String, usize)] {
        &self.entries
    }

    /// The TS keeps two `Map`s; the entries here are already sorted bytewise
    /// and numbered 1..n in that order, so the same lookups are a binary
    /// search and an index. Same answers, without an O(n) scan per load in
    /// the coupled loop.
    pub fn numeric(&self, stable_id: &str) -> PalletResult<usize> {
        self.entries
            .binary_search_by(|(stable, _)| bytewise_utf8_compare(stable, stable_id))
            .map(|index| self.entries[index].1)
            .map_err(|_| PalletError::sentence(format!("UNKNOWN_STABLE_ID: {stable_id}")))
    }

    pub fn stable(&self, numeric_id: usize) -> PalletResult<&str> {
        numeric_id
            .checked_sub(1)
            .and_then(|index| self.entries.get(index))
            .map(|(stable, _)| stable.as_str())
            .ok_or_else(|| PalletError::sentence(format!("UNKNOWN_NUMERIC_ID: {numeric_id}")))
    }
}
