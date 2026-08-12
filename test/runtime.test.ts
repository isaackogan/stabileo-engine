import { readFile } from "node:fs/promises";
import { describe, expect, test } from "vitest";
import {
  StabileoEngineError,
  initStabileoEngine,
  initStabileoEngineSync,
  version,
  type SolverInput,
} from "../src/index.js";

const wasmPath = new URL("../vendored/stabileo-engine.wasm", import.meta.url);
const packageJson = JSON.parse(
  await readFile(new URL("../package.json", import.meta.url), "utf8"),
);

function cantilever(): SolverInput {
  return {
    nodes: {
      "1": { id: 1, x: 0, z: 0 },
      "2": { id: 2, x: 5, z: 0 },
    },
    materials: { "1": { id: 1, e: 200e6, nu: 0.3 } },
    sections: { "1": { id: 1, a: 0.01, iz: 1e-4 } },
    elements: {
      "1": {
        id: 1,
        type: "frame",
        nodeI: 1,
        nodeJ: 2,
        materialId: 1,
        sectionId: 1,
      },
    },
    supports: { "1": { id: 1, nodeId: 1, type: "fixed" } },
    loads: [{ type: "nodal", data: { nodeId: 2, fx: 0, fz: -100, my: 0 } }],
  };
}

describe.sequential("StabileoEngine runtime", () => {
  test("normalizes initialization failures and permits a retry", async () => {
    await expect(initStabileoEngine({ wasm: new Uint8Array([0]) })).rejects.toMatchObject({
      name: "StabileoEngineError",
      code: "INITIALIZATION",
    });
  });

  test("initializes concurrently and reuses the same async and sync handle", async () => {
    const bytes = await readFile(wasmPath);
    const module = new WebAssembly.Module(bytes);

    const [first, second] = await Promise.all([
      initStabileoEngine({ wasm: bytes }),
      initStabileoEngine({ wasm: Promise.resolve(module) }),
    ]);

    expect(first).toBe(second);
    expect(await initStabileoEngine({ wasm: module })).toBe(first);
    expect(initStabileoEngineSync({ wasm: module })).toBe(first);
    expect(first.raw.solve_2d).toBeTypeOf("function");
    expect(first.raw.solve_pdelta_2d).toBeTypeOf("function");
    expect(version).toBe(packageJson.version);
  });

  test("solves a numerical 2D cantilever and exposes advanced JSON operations", async () => {
    const engine = await initStabileoEngine();
    const results = engine.solve2D(cantilever());
    const tip = results.displacements.find((value) => value.nodeId === 2);

    expect(tip).toBeDefined();
    expect(tip!.uz).toBeCloseTo(-0.0002083333, 7);
    expect(results.reactions[0]?.rz).toBeCloseTo(100, 8);

    const section = engine.analyzeSection({
      polygons: [{ vertices: [[0, 0], [2, 0], [2, 3], [0, 3]] }],
    });
    expect(section.a).toBeCloseTo(6, 12);
    expect(section.yc).toBeCloseTo(1, 12);
    expect(section.zc).toBeCloseTo(1.5, 12);
  });

  test("normalizes serialization and Rust/WASM failures", async () => {
    const engine = await initStabileoEngine();
    const circular: Record<string, unknown> = {};
    circular.self = circular;

    expect(() => engine.solvePDelta2D(circular as never, 10, 1e-8)).toThrowError(
      expect.objectContaining({ code: "SERIALIZATION" }),
    );
    expect(() => engine.solvePDelta2D({} as never, 10, 1e-8)).toThrowError(
      expect.objectContaining({ code: "WASM" }),
    );
    expect(StabileoEngineError).toBeTypeOf("function");
  });
});
