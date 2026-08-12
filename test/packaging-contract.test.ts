import { access, readFile, readdir } from "node:fs/promises";
import path from "node:path";
import { parse } from "yaml";
import { describe, expect, test } from "vitest";

const root = path.resolve(import.meta.dirname, "..");
const packageJson = JSON.parse(await readFile(path.join(root, "package.json"), "utf8"));

async function text(file: string) {
  return readFile(path.join(root, file), "utf8");
}

describe("package and workflow contracts", () => {
  test("declares a public ESM package with release and validation scripts", async () => {
    expect(packageJson).toMatchObject({
      name: "stabileo-engine",
      type: "module",
      license: "AGPL-3.0-only",
      packageManager: "pnpm@10.33.0",
      publishConfig: { access: "public" },
    });
    expect(packageJson.engines.node).toBe(">=22");
    expect(packageJson.scripts["release:pack"]).toBe("node ./src/scripts/check-package.mjs --output .");
  });

  test("every bundled sibling WASM URL resolves to the shipped branded asset", async () => {
    const dist = path.join(root, "dist");
    const emittedFiles = (await readdir(dist, { recursive: true })).sort();
    const javascriptFiles = emittedFiles.filter((file) => file.endsWith(".js"));
    const wasmFiles = emittedFiles.filter((file) => file.endsWith(".wasm"));
    const references: string[] = [];

    for (const javascriptFile of javascriptFiles) {
      const bundle = await readFile(path.join(dist, javascriptFile), "utf8");
      for (const match of bundle.matchAll(/new URL\((['"])([^'"]+\.wasm)\1,\s*import\.meta\.url\)/g)) {
        const reference = match[2];
        if (reference === undefined) throw new TypeError("WASM URL match omitted its path");
        references.push(reference);
      }
    }

    expect(wasmFiles).toEqual(["stabileo-engine.wasm"]);
    expect(references.length).toBeGreaterThan(0);
    expect(new Set(references)).toEqual(new Set(["./stabileo-engine.wasm"]));
    await Promise.all(
      Array.from(new Set(references), (reference) =>
        access(path.join(root, "dist", reference)),
      ),
    );
  });

  test("CI builds the checked-in snapshot and gates Dependabot auto-merge", async () => {
    const source = await text(".github/workflows/ci.yml");
    const workflow = parse(source);

    expect(workflow.jobs.checks).toBeDefined();
    expect(workflow.jobs.dependabot.needs).toBe("checks");
    expect(source).toContain("node-version: 22");
    expect(source).toContain("pnpm/action-setup@v6");
    expect(source).toContain("pnpm install --frozen-lockfile");
    expect(source).toContain("pnpm check");
    expect(source).not.toMatch(/(?:npm|pnpm)\s+(?:run\s+)?vendor\b/);
  });

  test("release uses OIDC, the exact tarball, and a dependent GitHub Release job", async () => {
    const source = await text(".github/workflows/release.yml");
    const packageChecker = await text("src/scripts/check-package.mjs");
    const releaseScript = await text("src/scripts/publish-github-release.sh");
    const workflow = parse(source);

    expect(workflow.jobs.publish.permissions["id-token"]).toBe("write");
    expect(workflow.jobs.release.needs).toBe("publish");
    expect(source).toContain("node-version: 24");
    expect(source).toContain("registry-url: https://registry.npmjs.org");
    expect(source).toContain("npm install --global npm@latest");
    expect(source).not.toContain("npm pack --json");
    expect(packageChecker).not.toContain('"--json"');
    expect(packageChecker).toContain("expectedFilename");
    expect(source).toContain('npm publish "$TARBALL" --access public');
    expect(source).not.toContain("npm version");
    expect(source).toContain("REGISTRY_SHASUM");
    expect(source).toContain('git push origin "$TAG"');
    expect(source.indexOf('git push origin "$TAG"')).toBeLessThan(
      source.indexOf('npm publish "$TARBALL" --access public'),
    );
    expect(source).toContain("actions/upload-artifact@v7");
    expect(source).toContain("actions/download-artifact@v7");
    expect(releaseScript).toContain("gh release create");
    expect(releaseScript).toContain("--verify-tag");
    expect(releaseScript).toContain("--generate-notes");
    expect(source).not.toMatch(/(?:npm|pnpm)\s+(?:run\s+)?vendor\b/);
  });

  test("documents provenance and does not claim orthotropic timber analysis", async () => {
    const readme = await text("README.md");

    expect(readme).toContain("npm run vendor");
    expect(readme).toContain("STABILEO_REVISION");
    expect(readme).toContain("AGPL-3.0-only");
    expect(readme).toContain("does not implement an orthotropic material model");
  });
});
