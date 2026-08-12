#!/usr/bin/env node

import { execFile, spawn } from "node:child_process";
import { access, copyFile, mkdtemp, readFile, rm } from "node:fs/promises";
import { tmpdir } from "node:os";
import path from "node:path";
import { pathToFileURL } from "node:url";
import { promisify } from "node:util";

const execFileAsync = promisify(execFile);
const root = path.resolve(import.meta.dirname, "../..");
const packageJson = JSON.parse(await readFile(path.join(root, "package.json"), "utf8"));
const binaryDirectory = path.join(root, "node_modules/.bin");
const outputFlag = process.argv.indexOf("--output");
const requestedOutput = outputFlag === -1 ? undefined : process.argv[outputFlag + 1];

if (outputFlag !== -1 && !requestedOutput) throw new Error("--output requires a directory");

async function commandBuffer(command, args) {
  return new Promise((resolve, reject) => {
    const child = spawn(command, args, { cwd: root, stdio: ["ignore", "pipe", "pipe"] });
    const stdout = [];
    let stderr = "";
    child.stdout.on("data", (chunk) => stdout.push(chunk));
    child.stderr.setEncoding("utf8").on("data", (chunk) => { stderr += chunk; });
    child.on("error", reject);
    child.on("close", (code) => {
      if (code === 0) resolve(Buffer.concat(stdout));
      else reject(new Error(`${command} exited ${code}: ${stderr}`));
    });
  });
}

const temporaryDirectory = await mkdtemp(path.join(tmpdir(), "stabileo-package-"));
try {
  const expectedFilename = `${packageJson.name}-${packageJson.version}.tgz`;
  const tarball = path.join(temporaryDirectory, expectedFilename);
  await execFileAsync("npm", ["pack", "--pack-destination", temporaryDirectory], {
    cwd: root,
    maxBuffer: 8 * 1024 * 1024,
  });
  try {
    await access(tarball);
  } catch {
    throw new Error(`npm pack did not produce the expected tarball ${expectedFilename}`);
  }
  const listing = (await commandBuffer("tar", ["-tf", tarball])).toString("utf8").trim().split("\n");
  const required = [
    "package/package.json",
    "package/dist/index.js",
    "package/dist/index.d.ts",
    "package/dist/index.js.map",
    "package/dist/stabileo-engine.wasm",
    "package/README.md",
    "package/LICENSE",
  ];
  for (const entry of required) {
    if (!listing.includes(entry)) throw new Error(`tarball is missing ${entry}`);
  }
  const wasmEntries = listing.filter((entry) => entry.endsWith(".wasm"));
  if (wasmEntries.length !== 1 || wasmEntries[0] !== "package/dist/stabileo-engine.wasm") {
    throw new Error(`tarball must contain exactly one branded WASM, found: ${wasmEntries.join(", ")}`);
  }
  if (listing.some((entry) => entry.startsWith("package/vendored/") || entry.startsWith("package/src/"))) {
    throw new Error("tarball contains development-only Rust or TypeScript sources");
  }

  const [packedWasm, builtWasm] = await Promise.all([
    commandBuffer("tar", ["-xOf", tarball, "package/dist/stabileo-engine.wasm"]),
    readFile(path.join(root, "dist/stabileo-engine.wasm")),
  ]);
  if (!packedWasm.equals(builtWasm)) throw new Error("packed WASM differs from dist/stabileo-engine.wasm");

  await execFileAsync(path.join(binaryDirectory, "publint"), [tarball, "--strict"], { cwd: root });
  await execFileAsync(path.join(binaryDirectory, "attw"), [tarball, "--profile", "esm-only"], { cwd: root });

  const bundleUrl = pathToFileURL(path.join(root, "dist/index.js")).href;
  await execFileAsync(process.execPath, [
    "--input-type=module",
    "--eval",
    `const sdk = await import(${JSON.stringify(bundleUrl)}); const engine = await sdk.initStabileoEngine(); const value = engine.analyzeSection({ polygons: [{ vertices: [[0, 0], [2, 0], [2, 3], [0, 3]] }] }); if (value.a !== 6) throw new Error('built Node package failed its WASM smoke test');`,
  ], { cwd: root });

  const verifiedTarball = requestedOutput
    ? path.join(path.resolve(root, requestedOutput), expectedFilename)
    : tarball;
  if (requestedOutput) await copyFile(tarball, verifiedTarball);
  process.stdout.write(`Verified ${verifiedTarball}\n`);
} finally {
  await rm(temporaryDirectory, { recursive: true, force: true });
}
