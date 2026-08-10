#!/usr/bin/env node

import { readFile, writeFile } from "node:fs/promises";
import path from "node:path";

const root = path.resolve(import.meta.dirname, "../..");
const packageJson = JSON.parse(await readFile(path.join(root, "package.json"), "utf8"));
const version = process.argv[2] ?? packageJson.version;

if (version !== packageJson.version) {
  throw new Error(`package.json is ${packageJson.version}, not ${version}`);
}
if (!/^\d+\.\d+\.\d+(?:-[0-9A-Za-z.-]+)?$/.test(version)) {
  throw new Error(`invalid package version: ${version}`);
}

await writeFile(
  path.join(root, "src/version.ts"),
  `// Synchronized from package.json by src/scripts/sync-version.mjs.\nexport const version = ${JSON.stringify(version)};\n`,
);
