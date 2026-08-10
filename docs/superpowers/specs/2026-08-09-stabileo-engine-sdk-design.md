# Stabileo Engine SDK Design

## Goal

Package the upstream Stabileo Rust engine as the unscoped ESM npm package
`stabileo-engine`, with a checked-in upstream source snapshot, a separately
shipped WebAssembly asset, compiler-derived TypeScript bindings, and matching
npm and GitHub releases.

## Source and build boundary

`npm run vendor` is the only operation that downloads Stabileo. It sparse
checks out `engine/`, preserves the upstream AGPL license, records the resolved
revision, and atomically replaces `vendored/stabileo`. CI and release builds
must never call it.

`npm run build` compiles only the checked-in snapshot. wasm-pack emits web
bindings in a temporary directory; the paired JavaScript glue is staged for
the SDK build and the binary is atomically copied to
`vendored/stabileo-engine.wasm`. tsdown bundles the glue and SDK while copying
the binary to `dist/stabileo-engine.wasm` as a sibling asset.

## Bindings and runtime API

Rust and Serde are the schema source of truth. A rustdoc-JSON generator follows
compiler-resolved types from a small ABI manifest and generates canonical
TypeScript models. The manifest contains only information erased at the WASM
boundary: operation names, transports, scalar arguments, and Rust input/output
roots. It must cover every wasm-bindgen export. A semantic lock makes changes
reviewable, and unsupported type constructs fail generation.

`initStabileoEngine()` returns one cached engine handle. It automatically loads
the sibling asset with fetch in web runtimes and filesystem bytes in Node, and
accepts caller-supplied bytes, responses, URLs, or `WebAssembly.Module` values.
A synchronous initializer requires bytes or a compiled module. The handle has
typed camelCase methods and a `raw` namespace with the original snake_case
exports. All boundary failures are normalized as `StabileoEngineError`.

## Distribution and licensing

The package is ESM-only, supports Node 22+, and is AGPL-3.0-only because it
distributes a derivative of Stabileo. The npm tarball contains JavaScript,
declarations, README, license, and the sibling WASM.

The manual release workflow publishes one generated `.tgz` to npm, commits and
tags the version, then hands the exact tarball and WASM to a dependent GitHub
Release job. That job creates generated notes and attaches both assets without
silently replacing mismatched existing assets.
