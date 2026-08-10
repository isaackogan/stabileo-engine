# Stabileo Engine SDK Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and release a typed, cross-runtime npm SDK around the vendored Stabileo WebAssembly engine.

**Architecture:** A local-only sparse vendor step records an upstream Rust snapshot. Build scripts compile that snapshot with wasm-pack and derive TypeScript schemas from rustdoc JSON; tsdown bundles a typed engine wrapper and ships the WASM as a sibling npm asset. CI verifies the checked-in snapshot, while release publishes the same tarball and WASM to npm and GitHub.

**Tech Stack:** POSIX sh, Rust nightly, wasm-pack 0.13.1, Node.js 22+, TypeScript, tsdown, Vitest, Playwright, pnpm, GitHub Actions.

## Global Constraints

- Package name: `stabileo-engine`; repository: `isaackogan/stabileo-engine`.
- License: `AGPL-3.0-only`; preserve the upstream license and revision.
- Only `npm run vendor` may download Stabileo; workflows compile checked-in source.
- Output binary: `vendored/stabileo-engine.wasm`; published sibling asset: `dist/stabileo-engine.wasm`.
- ESM-only; Node.js `>=22`; pnpm `10.33.0`.
- Generated type surfaces must not silently use `any` or `unknown` for unsupported Rust/Serde constructs.

---

### Task 1: Package and test harness

- [x] Add package metadata, strict TypeScript, tsdown, Vitest, Playwright, ignore rules, and manager-neutral scripts.
- [x] Install dependencies and establish the empty-project test baseline.

### Task 2: Vendoring

- [x] Write failing integration tests against a local Git fixture for sparse content, revision/license preservation, and replacement behavior.
- [x] Implement the POSIX vendor script and observe the tests pass.
- [x] Run the package vendor command against `lambdaclass/stabileo` and inspect the snapshot.

### Task 3: WASM build and bindings generation

- [x] Write failing script tests for paired glue/binary placement and failure atomicity.
- [x] Implement the wasm-pack build script, then compile the real vendored crate.
- [x] Write failing rustdoc fixture tests for Serde renames, tagged enums, collections, optionality, unsupported shapes, semantic locking, and export coverage.
- [x] Implement the generator and manifest, generate the real bindings, and approve the initial semantic lock.

### Task 4: Runtime SDK

- [x] Write failing tests for async/sync/idempotent initialization, typed solver methods, raw parity, JSON adapters, and normalized errors.
- [x] Implement the engine handle and generated adapters.
- [x] Build with tsdown and run a real cantilever solve in Node and a browser smoke test.

### Task 5: Packaging and automation

- [x] Write failing package/workflow contract tests.
- [x] Implement CI and manual release workflows, including exact npm tarball publication and dependent GitHub Release assets.
- [x] Add package validation, README, and AGPL/provenance documentation.
- [x] Run the complete check, inspect the dry-run tarball, and verify the WASM inside it matches the release asset.
