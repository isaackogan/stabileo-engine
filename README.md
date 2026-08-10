# stabileo-engine

Typed ESM bindings for the [Stabileo](https://github.com/lambdaclass/stabileo) structural-analysis engine. The package ships compiler-derived TypeScript models, bundled wasm-bindgen glue, and `stabileo-engine.wasm` as a sibling asset.

```sh
npm install stabileo-engine
```

```ts
import { initStabileoEngine } from "stabileo-engine";

const engine = await initStabileoEngine();
const results = engine.solve2D({
  nodes: {
    "1": { id: 1, x: 0, z: 0 },
    "2": { id: 2, x: 5, z: 0 },
  },
  materials: { "1": { id: 1, e: 200e6, nu: 0.3 } },
  sections: { "1": { id: 1, a: 0.01, iz: 1e-4 } },
  elements: {
    "1": { id: 1, type: "frame", nodeI: 1, nodeJ: 2, materialId: 1, sectionId: 1 },
  },
  supports: { "1": { id: 1, nodeId: 1, type: "fixed" } },
  loads: [{ type: "nodal", data: { nodeId: 2, fx: 0, fz: -100, my: 0 } }],
});
```

## Initialization

`initStabileoEngine()` loads `stabileo-engine.wasm` relative to the bundled JavaScript. It reads the asset from the filesystem in Node and fetches it in browsers and Workers. The WASM is included in the npm package, but is not base64-inlined into JavaScript.

Custom bytes, responses, URLs, and precompiled modules are supported:

```ts
const engine = await initStabileoEngine({ wasm: fetch(myWasmUrl) });

const syncEngine = initStabileoEngineSync({
  wasm: new WebAssembly.Module(wasmBytes),
});
```

Successful initialization is idempotent: async, concurrent, repeated, and sync calls share one engine handle. Every upstream operation has a camel-cased, object-in/object-out method. `engine.raw` exposes the corresponding original snake_case wasm-bindgen functions for low-level JSON/value transport.

## Generated types and vendoring

The checked-in Rust snapshot is recorded in [`vendored/STABILEO_REVISION`](./vendored/STABILEO_REVISION). Updating it is an explicit local action:

```sh
npm run vendor
npm run build:wasm
npm run bindings:update
```

`npm run vendor` sparse-clones only upstream `engine/`, preserves its AGPL license, and atomically replaces the snapshot. `STABILEO_REF` can select a tag or commit. CI and release workflows never call the vendor command; they build only the reviewed source already present in this repository.

Bindings come from nightly `cargo rustdoc` JSON and resolved Serde attributes. The small manifest records only exported Rust roots, scalar ABI parameters, transport, and method names. Generation fails on unsupported constructs, missing wasm-bindgen exports, or changes to the semantic bindings lock. This prevents silent `any` fallbacks and schema drift.

## Timber scope

Stabileo includes NDS timber member design checks and supports analysis with user-supplied effective section/material properties. It does not implement an orthotropic material model: the solver accepts one `E` and one Poisson ratio and derives isotropic shear modulus. CLT, plywood, or other directional materials therefore require precomputed effective properties; this SDK does not claim orthotropic timber or laminate analysis.

## Development

The pinned toolchain is nightly Rust with `rust-src`, `wasm32-unknown-unknown`, wasm-pack 0.13.1, Node 22+, and pnpm 10.33.0.

```sh
pnpm install --frozen-lockfile
pnpm check
pnpm release:pack
```

`pnpm check` compiles the checked-in Rust, verifies the semantic bindings lock, builds with tsdown, runs Node and browser integration tests, checks declarations with TypeScript, runs publint and Are the Types Wrong, and validates the npm tarball. `pnpm release:pack` leaves the verified `stabileo-engine-<version>.tgz` in the project root.

## License and provenance

This derivative package and the vendored Stabileo engine are distributed under **AGPL-3.0-only**. See [`LICENSE`](./LICENSE), the vendored copy at [`vendored/stabileo/LICENSE`](./vendored/stabileo/LICENSE), and the pinned upstream revision at [`vendored/STABILEO_REVISION`](./vendored/STABILEO_REVISION).

GitHub releases attach the exact npm tarball plus a byte-identical `stabileo-engine.wasm`. Trusted npm publication uses GitHub Actions OIDC; public releases receive npm provenance automatically.
