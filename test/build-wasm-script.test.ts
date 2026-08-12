import { afterEach, describe, expect, test } from "vitest";
import { chmod, cp, mkdir, mkdtemp, readFile, rm, writeFile } from "node:fs/promises";
import { execFile } from "node:child_process";
import { promisify } from "node:util";
import { tmpdir } from "node:os";
import path from "node:path";

const execFileAsync = promisify(execFile);
const temporaryDirectories: string[] = [];

async function createFixture(): Promise<{ root: string; script: string; bin: string }> {
  const root = await mkdtemp(path.join(tmpdir(), "stabileo-wasm-build-"));
  temporaryDirectories.push(root);
  const scripts = path.join(root, "src", "scripts");
  const engine = path.join(root, "vendored", "stabileo");
  const bin = path.join(root, "bin");
  await Promise.all([
    mkdir(scripts, { recursive: true }),
    mkdir(engine, { recursive: true }),
    mkdir(bin, { recursive: true }),
  ]);
  await writeFile(path.join(root, "package.json"), "{}\n");
  await writeFile(path.join(engine, "Cargo.toml"), "[package]\nname = \"fixture\"\nversion = \"0.1.0\"\n");
  await writeFile(path.join(root, "vendored", "STABILEO_REVISION"), "0123456789abcdef\n");
  const script = path.join(scripts, "build-wasm.sh");
  await cp(path.resolve("src/scripts/build-wasm.sh"), script);

  const rustup = path.join(bin, "rustup");
  await writeFile(rustup, `#!/bin/sh
case "$1 $2" in
  "run nightly") echo "rustc 1.99.0-nightly" ;;
  "target list") echo "wasm32-unknown-unknown" ;;
  "component list") echo "rust-src-aarch64-apple-darwin (installed)" ;;
  *) exit 1 ;;
esac
`);
  await chmod(rustup, 0o755);

  const wasmPack = path.join(bin, "wasm-pack");
  await writeFile(wasmPack, `#!/bin/sh
if [ "\${FAKE_WASM_PACK_FAIL:-0}" = "1" ]; then exit 42; fi
if [ "\${1:-}" = "--version" ]; then echo "wasm-pack 0.13.1"; exit 0; fi
out_dir=""
out_name=""
while [ "$#" -gt 0 ]; do
  case "$1" in
    --out-dir) out_dir=$2; shift 2 ;;
    --out-name) out_name=$2; shift 2 ;;
    *) shift ;;
  esac
done
mkdir -p "$out_dir"
printf 'wasm-bytes' > "$out_dir/\${out_name}_bg.wasm"
case "\${FAKE_WASM_GLUE_MODE:-one}" in
  zero) printf 'export default async function init() {}\n' > "$out_dir/\${out_name}.js" ;;
  one) printf "export default async function init(module_or_path) { if (module_or_path === undefined) module_or_path = new URL('stabileo_engine_bg.wasm', import.meta.url); }\n" > "$out_dir/\${out_name}.js" ;;
  multiple) printf "const first = new URL('stabileo_engine_bg.wasm', import.meta.url); const second = new URL('stabileo_engine_bg.wasm', import.meta.url); export default async function init() {}\n" > "$out_dir/\${out_name}.js" ;;
  *) exit 43 ;;
esac
printf 'export default function init(): Promise<void>;\n' > "$out_dir/\${out_name}.d.ts"
`);
  await chmod(wasmPack, 0o755);

  return { root, script, bin };
}

afterEach(async () => {
  await Promise.all(temporaryDirectories.splice(0).map((directory) => rm(directory, { recursive: true, force: true })));
});

describe("build-wasm.sh", () => {
  test("publishes one matched WASM and wasm-bindgen glue set", async () => {
    const fixture = await createFixture();

    await execFileAsync("sh", [fixture.script], {
      cwd: fixture.root,
      env: {
        ...process.env,
        PATH: `${fixture.bin}:${process.env.PATH ?? ""}`,
      },
    });

    expect(await readFile(path.join(fixture.root, "vendored", "stabileo-engine.wasm"), "utf8"))
      .toBe("wasm-bytes");
    const generatedGlue = await readFile(
      path.join(fixture.root, "src", "generated", "wasm", "stabileo_engine.js"),
      "utf8",
    );
    expect(generatedGlue).toContain("async function init");
    expect(generatedGlue).toContain("new URL('./stabileo-engine.wasm', import.meta.url)");
    expect(generatedGlue).not.toContain("stabileo_engine_bg.wasm");
    expect(await readFile(path.join(fixture.root, "src", "generated", "wasm", "stabileo_engine.d.ts"), "utf8"))
      .toContain("Promise<void>");
  });

  test("leaves an existing matched output set unchanged when compilation fails", async () => {
    const fixture = await createFixture();
    const generated = path.join(fixture.root, "src", "generated", "wasm");
    await mkdir(generated, { recursive: true });
    await writeFile(path.join(fixture.root, "vendored", "stabileo-engine.wasm"), "old-wasm");
    await writeFile(path.join(generated, "stabileo_engine.js"), "old-js");
    await writeFile(path.join(generated, "stabileo_engine.d.ts"), "old-dts");

    await expect(execFileAsync("sh", [fixture.script], {
      cwd: fixture.root,
      env: {
        ...process.env,
        PATH: `${fixture.bin}:${process.env.PATH ?? ""}`,
        FAKE_WASM_PACK_FAIL: "1",
      },
    })).rejects.toMatchObject({ code: 42 });

    expect(await readFile(path.join(fixture.root, "vendored", "stabileo-engine.wasm"), "utf8")).toBe("old-wasm");
    expect(await readFile(path.join(generated, "stabileo_engine.js"), "utf8")).toBe("old-js");
    expect(await readFile(path.join(generated, "stabileo_engine.d.ts"), "utf8")).toBe("old-dts");
  });

  test.each(["zero", "multiple"])(
    "fails closed when wasm-bindgen emits %s fallback references",
    async (mode) => {
      const fixture = await createFixture();

      await expect(execFileAsync("sh", [fixture.script], {
        cwd: fixture.root,
        env: {
          ...process.env,
          PATH: `${fixture.bin}:${process.env.PATH ?? ""}`,
          FAKE_WASM_GLUE_MODE: mode,
        },
      })).rejects.toMatchObject({
        stderr: expect.stringContaining("expected exactly one generated WASM URL"),
      });
    },
  );
});
