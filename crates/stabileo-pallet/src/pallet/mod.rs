//! The pallet-top contact modules: the projection that turns package contact
//! patches into frame loads plus a rigid contact-face system, and the recovery
//! that reads the solved deck back out as a per-contact rigid-body motion.
//!
//! Both are literal ports of the application's TypeScript reference
//! (`packages/analysis/pallet/src/contact-projection.ts` and
//! `.../top-response.ts`); see `PORTING.md` for the rules they follow.

pub mod projection;
pub mod top_response;

/// `compareCanonicalUtf8` from the contracts package: order two strings by
/// their UTF-8 BYTES, shorter-is-smaller on a common prefix.
///
/// The reference encodes both sides and compares byte by byte, returning the
/// length difference when one is a prefix of the other — which is exactly
/// Rust's `[u8]` lexicographic ordering, because `str` is already UTF-8. The
/// sorts this feeds are identity-bearing (the application hashes the sorted
/// arrays), so the ordering is contract, not taste.
pub fn compare_canonical_utf8(left: &str, right: &str) -> std::cmp::Ordering {
    left.as_bytes().cmp(right.as_bytes())
}

#[cfg(test)]
mod tests {
    use super::compare_canonical_utf8;
    use std::cmp::Ordering;

    #[test]
    fn orders_by_utf8_bytes_with_length_as_the_tiebreak() {
        assert_eq!(compare_canonical_utf8("a", "b"), Ordering::Less);
        assert_eq!(compare_canonical_utf8("ab", "a"), Ordering::Greater);
        assert_eq!(compare_canonical_utf8("a", "a"), Ordering::Equal);
        // Byte order, not code-point-in-UTF-16 order: "Z" (0x5A) precedes the
        // lowercase letters, and a two-byte scalar sorts after every ASCII one.
        assert_eq!(compare_canonical_utf8("Z", "a"), Ordering::Less);
        assert_eq!(compare_canonical_utf8("é", "z"), Ordering::Greater);
    }
}
