# Porting conventions — TypeScript reference → stabileo-pallet

The TypeScript implementation in the consuming application is the REFERENCE.
This crate is a literal port, gated by an end-to-end golden comparison against
the TS pipeline on the committed reproducer fixtures. Until that gate is
green, fidelity beats taste.

Rules:

1. **Translate literally.** Same function boundaries, same statement order,
   same accumulation order in folds/reductions (f64 addition is not
   associative and the gate compares numbers). Do not "improve" logic, do not
   reorder guards, do not simplify algebra.
2. **Keep the essays.** The TS files carry doc comments explaining measured
   decisions. Port each comment with its function — they are the provenance
   of every non-obvious constant and guard.
3. **Numbers are f64.** No f32 anywhere. Integer indices are `usize`.
4. **Nullable → `Option<T>`.** TS `x ?? fallback` → `x.unwrap_or(fallback)`;
   preserve exactly which side is the fallback.
5. **Serde naming:** every struct that crosses JSON derives
   `#[serde(rename_all = "camelCase")]` unless a field carries an explicit
   rename; unknown/untouched fields of app-schema objects are preserved with
   `#[serde(flatten)] pub extra: serde_json::Map<String, serde_json::Value>`
   so a state the logic does not touch round-trips byte-equivalent.
6. **Error sentences are contract.** TS `throw new TypeError("CODE: message")`
   → `return Err(PalletError::new("CODE", message))` with the SAME code and
   the same message text (interpolations included). Consumers pattern-match
   these strings.
7. **No hashing inside the loop.** The TS per-round `sha256CanonicalEnvelope`
   identity stamps are application identity, recomputed app-side on the
   returned terminal state; the port carries the fields but computes no
   hashes.
8. **Ambiguity stops the port.** If the TS is unclear or two readings are
   possible, do not pick one — leave a `// PORT-QUESTION:` comment and make
   the item fail loudly, then surface it.
