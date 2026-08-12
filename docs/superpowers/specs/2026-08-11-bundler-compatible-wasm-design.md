# Bundler-Compatible WASM Release Design

## Problem

`stabileo-engine@0.1.0` ships `dist/stabileo-engine.wasm`, but the bundled
wasm-bindgen initializer still contains a fallback URL for
`stabileo_engine_bg.wasm`. Native ESM execution succeeds because the wrapper
always supplies the shipped asset explicitly. Static bundlers such as Next.js
Turbopack resolve every sibling `new URL(..., import.meta.url)` expression and
reject the package because the generated filename is absent.

## Design

Normalize the exact wasm-bindgen fallback URL in the checked-in generated glue
from `stabileo_engine_bg.wasm` to `./stabileo-engine.wasm` immediately after a
successful WASM build. The build must fail closed unless it finds exactly one
generated fallback reference. The public initialization API, the branded
single-file WASM distribution, and explicit caller-supplied WASM bytes remain
unchanged.

## Verification

- A build-script fixture proves the generated glue is normalized.
- A packaging test extracts every sibling WASM URL from `dist/index.js` and
  proves each URL resolves to a file shipped in `dist`.
- Existing Node and browser integration tests continue to initialize the SDK
  and execute real Stabileo operations.
- The npm tarball remains strict-publint/ATTW clean and contains one branded
  WASM payload.
- FastPallet must build with Turbopack and complete its real browser E2E flow
  against `stabileo-engine@0.1.1`.

## Release

Commit the fix to `main`, dispatch the repository's `Release` workflow with a
`patch` bump, wait for npm trusted publication and the GitHub Release job, then
pin FastPallet to the published `0.1.1` artifact and refresh its frozen runtime
hashes from those exact bytes.
