#!/usr/bin/env node

import { execFile } from "node:child_process";
import { mkdtemp, readFile, rm, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import path from "node:path";
import { promisify } from "node:util";
import { bindingsLock, generateBindings } from "./generator.mjs";

const execFileAsync = promisify(execFile);
const root = path.resolve(import.meta.dirname, "../..");

function parseArguments(argv) {
  const command = argv.shift();
  if (command !== "generate" && command !== "update") {
    throw new Error("usage: cli.mjs <generate|update> [--rustdoc file --wasm-dts file --manifest file --output file --lock file]");
  }
  const options = {};
  while (argv.length) {
    const flag = argv.shift();
    const value = argv.shift();
    if (!flag?.startsWith("--") || !value) throw new Error(`invalid argument: ${flag ?? ""}`);
    options[flag.slice(2)] = path.resolve(value);
  }
  return {
    command,
    rustdoc: options.rustdoc,
    wasmDts: options["wasm-dts"] ?? path.join(root, "src/generated/wasm/stabileo_engine.d.ts"),
    manifest: options.manifest ?? path.join(root, "src/typegen/manifest.json"),
    output: options.output ?? path.join(root, "src/generated/bindings.ts"),
    lock: options.lock ?? path.join(root, "src/typegen/bindings.lock.json"),
  };
}

async function generateRustdoc() {
  const directory = await mkdtemp(path.join(tmpdir(), "stabileo-rustdoc-"));
  try {
    await execFileAsync("cargo", [
      "+nightly", "rustdoc", "--locked", "--lib",
      "--manifest-path", path.join(root, "vendored/stabileo/Cargo.toml"),
      "--target-dir", path.join(directory, "target"),
      "--", "-Z", "unstable-options", "--output-format", "json",
    ], { cwd: root, maxBuffer: 16 * 1024 * 1024 });
    const rustdocPath = path.join(directory, "target/doc/dedaliano_engine.json");
    return { directory, contents: await readFile(rustdocPath, "utf8") };
  } catch (error) {
    await rm(directory, { recursive: true, force: true });
    throw error;
  }
}

async function main() {
  const options = parseArguments(process.argv.slice(2));
  let temporaryRustdoc;
  try {
    const rustdocContents = options.rustdoc
      ? await readFile(options.rustdoc, "utf8")
      : (temporaryRustdoc = await generateRustdoc()).contents;
    const [wasmDts, manifestContents] = await Promise.all([
      readFile(options.wasmDts, "utf8"),
      readFile(options.manifest, "utf8"),
    ]);
    const generated = generateBindings(JSON.parse(rustdocContents), wasmDts, JSON.parse(manifestContents));
    const lock = bindingsLock(generated.semantic);
    const serializedLock = `${JSON.stringify(lock, null, 2)}\n`;

    if (options.command === "generate") {
      let currentLock;
      try {
        currentLock = JSON.parse(await readFile(options.lock, "utf8"));
      } catch {
        throw new Error("bindings lock is missing; run npm run bindings:update");
      }
      if (currentLock.semanticSha256 !== lock.semanticSha256) {
        throw new Error("bindings changed; run npm run bindings:update");
      }
    }

    await writeFile(options.output, generated.source);
    if (options.command === "update") await writeFile(options.lock, serializedLock);
  } finally {
    if (temporaryRustdoc) await rm(temporaryRustdoc.directory, { recursive: true, force: true });
  }
}

main().catch((error) => {
  process.stderr.write(`${error instanceof Error ? error.message : String(error)}\n`);
  process.exitCode = 1;
});
