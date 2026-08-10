import { afterEach, describe, expect, test } from "vitest";
import { mkdtemp, readFile, rm, writeFile } from "node:fs/promises";
import { spawn } from "node:child_process";
import { tmpdir } from "node:os";
import path from "node:path";

const temporaryDirectories: string[] = [];

function field(id: number, name: string, type: unknown, attrs: string[] = []) {
  return {
    id,
    crate_id: 0,
    name,
    visibility: "public",
    attrs: attrs.map((other) => ({ other })),
    inner: { struct_field: type },
  };
}

function struct(id: number, name: string, fields: number[], attrs: string[] = []) {
  return {
    id,
    crate_id: 0,
    name,
    visibility: "public",
    attrs: attrs.map((other) => ({ other })),
    inner: {
      struct: {
        kind: { plain: { fields, has_stripped_fields: false } },
        generics: { params: [], where_predicates: [] },
        impls: [],
      },
    },
  };
}

function variant(id: number, name: string, kind: unknown, attrs: string[] = []) {
  return {
    id,
    crate_id: 0,
    name,
    visibility: "public",
    attrs: attrs.map((other) => ({ other })),
    inner: { variant: { kind, discriminant: null } },
  };
}

function rustdocFixture() {
  const pathType = (name: string, id: number, args: unknown = null) => ({
    resolved_path: { path: name, id, args },
  });
  const genericArgs = (...types: unknown[]) => ({
    angle_bracketed: {
      args: types.map((type) => ({ type })),
      constraints: [],
    },
  });

  return {
    root: 0,
    crate_version: "0.1.0",
    includes_private: false,
    format_version: 61,
    target: { triple: "test" },
    paths: {},
    external_crates: {},
    index: {
      "1": struct(1, "SolverNode", [2, 3], ["#[serde(rename_all = \"camelCase\")]" ]),
      "2": field(2, "node_id", { primitive: "usize" }),
      "3": field(3, "x", { primitive: "f64" }),
      "4": struct(4, "NodalLoad", [5]),
      "5": field(5, "force", { primitive: "f64" }),
      "6": {
        id: 6,
        crate_id: 0,
        name: "SolverLoad",
        visibility: "public",
        attrs: [{ other: "#[serde(tag = \"type\", content = \"data\")]" }],
        inner: {
          enum: {
            generics: { params: [], where_predicates: [] },
            has_stripped_variants: false,
            variants: [7],
            impls: [],
          },
        },
      },
      "7": variant(7, "Nodal", { tuple: [14] }, ["#[serde(rename = \"nodal\")]" ]),
      "8": struct(8, "SolverInput", [9, 10, 11, 15], ["#[serde(rename_all = \"camelCase\")]" ]),
      "9": field(9, "nodes", pathType("std::collections::HashMap", 90, genericArgs(
        pathType("String", 91),
        pathType("SolverNode", 1),
      ))),
      "10": field(10, "loads", pathType("Vec", 92, genericArgs(pathType("SolverLoad", 6)))),
      "11": field(11, "max_steps", pathType("Option", 93, genericArgs({ primitive: "usize" })), ["#[serde(default)]"]),
      "12": struct(12, "AnalysisResults", [13]),
      "13": field(13, "converged", { primitive: "bool" }),
      "14": field(14, "0", pathType("NodalLoad", 4)),
      "15": field(15, "load_kind", pathType("LoadKind", 16)),
      "16": {
        id: 16,
        crate_id: 0,
        name: "LoadKind",
        visibility: "public",
        attrs: [{ other: "#[serde(rename_all = \"lowercase\")]" }],
        inner: {
          enum: {
            generics: { params: [], where_predicates: [] },
            has_stripped_variants: false,
            variants: [17],
            impls: [],
          },
        },
      },
      "17": variant(17, "None", "plain"),
    },
  };
}

async function runTypegen(args: string[]) {
  return new Promise<{ code: number | null; stdout: string; stderr: string }>((resolve) => {
    const child = spawn(process.execPath, [path.resolve("src/typegen/cli.mjs"), ...args], {
      cwd: process.cwd(),
      stdio: ["ignore", "pipe", "pipe"],
    });
    let stdout = "";
    let stderr = "";
    child.stdout.setEncoding("utf8").on("data", (chunk) => { stdout += chunk; });
    child.stderr.setEncoding("utf8").on("data", (chunk) => { stderr += chunk; });
    child.on("close", (code) => resolve({ code, stdout, stderr }));
  });
}

async function fixtureFiles(manifestOverride?: object) {
  const directory = await mkdtemp(path.join(tmpdir(), "stabileo-typegen-"));
  temporaryDirectories.push(directory);
  const rustdoc = path.join(directory, "rustdoc.json");
  const dts = path.join(directory, "stabileo_engine.d.ts");
  const manifest = path.join(directory, "manifest.json");
  const output = path.join(directory, "bindings.ts");
  const lock = path.join(directory, "bindings.lock.json");
  await writeFile(rustdoc, `${JSON.stringify(rustdocFixture())}\n`);
  await writeFile(dts, [
    "export function solve_2d(input: any): any;",
    "export function solve_pdelta_2d(json: string, max_iter: number, tolerance: number): string;",
  ].join("\n"));
  await writeFile(manifest, `${JSON.stringify(manifestOverride ?? {
    operations: {
      solve_2d: {
        method: "solve2D",
        transport: "value",
        input: "SolverInput",
        output: "AnalysisResults",
        parameters: [],
      },
      solve_pdelta_2d: {
        method: "solvePDelta2D",
        transport: "json",
        input: "SolverInput",
        output: "AnalysisResults",
        parameters: [
          { raw: "max_iter", name: "maxIter", type: "number" },
          { raw: "tolerance", name: "tolerance", type: "number" },
        ],
      },
    },
  })}\n`);
  return { directory, rustdoc, dts, manifest, output, lock };
}

function argumentsFor(command: "generate" | "update", files: Awaited<ReturnType<typeof fixtureFiles>>) {
  return [
    command,
    "--rustdoc", files.rustdoc,
    "--wasm-dts", files.dts,
    "--manifest", files.manifest,
    "--output", files.output,
    "--lock", files.lock,
  ];
}

afterEach(async () => {
  await Promise.all(temporaryDirectories.splice(0).map((directory) => rm(directory, { recursive: true, force: true })));
});

describe("compiler-derived binding generator", () => {
  test("emits Serde-canonical records, tagged unions, and typed operations", async () => {
    const files = await fixtureFiles();

    const result = await runTypegen(argumentsFor("update", files));

    expect(result).toMatchObject({ code: 0, stderr: "" });
    const generated = await readFile(files.output, "utf8");
    expect(generated).toContain("export interface SolverNode");
    expect(generated).toContain("nodeId: number;");
    expect(generated).toContain('export type SolverLoad = { type: "nodal"; data: NodalLoad };');
    expect(generated).toContain("nodes: Record<string, SolverNode>;");
    expect(generated).toContain('export type LoadKind = "none";');
    expect(generated).toContain("maxSteps?: number | null;");
    expect(generated).toContain("solve2D(input: SolverInput): AnalysisResults;");
    expect(generated).toContain("solvePDelta2D(input: SolverInput, maxIter: number, tolerance: number): AnalysisResults;");
  });

  test("fails coverage when a wasm-bindgen export is missing from the manifest", async () => {
    const files = await fixtureFiles({
      operations: {
        solve_2d: {
          method: "solve2D",
          transport: "value",
          input: "SolverInput",
          output: "AnalysisResults",
          parameters: [],
        },
      },
    });

    const result = await runTypegen(argumentsFor("update", files));

    expect(result.code).toBe(1);
    expect(result.stderr).toContain("missing manifest entries: solve_pdelta_2d");
  });

  test("detects semantic drift and refuses unsupported rustdoc types", async () => {
    const files = await fixtureFiles();
    expect((await runTypegen(argumentsFor("update", files))).code).toBe(0);

    const changed = rustdocFixture();
    changed.index["13"].inner.struct_field = { primitive: "f64" };
    await writeFile(files.rustdoc, `${JSON.stringify(changed)}\n`);
    const drift = await runTypegen(argumentsFor("generate", files));
    expect(drift.code).toBe(1);
    expect(drift.stderr).toContain("bindings changed; run npm run bindings:update");

    changed.index["13"].inner.struct_field = { function_pointer: { sig: {} } };
    await writeFile(files.rustdoc, `${JSON.stringify(changed)}\n`);
    const unsupported = await runTypegen(argumentsFor("update", files));
    expect(unsupported.code).toBe(1);
    expect(unsupported.stderr).toContain("unsupported rustdoc type");
  });
});
