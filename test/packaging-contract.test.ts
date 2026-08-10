import { readFile } from "node:fs/promises";
import path from "node:path";
import { parse } from "yaml";
import { describe, expect, test } from "vitest";

const root = path.resolve(import.meta.dirname, "..");

async function text(file: string) {
  return readFile(path.join(root, file), "utf8");
}

describe("package and workflow contracts", () => {
  test("declares a public ESM package with release and validation scripts", async () => {
    const packageJson = JSON.parse(await text("package.json"));

    expect(packageJson).toMatchObject({
      name: "stabileo-engine",
      version: "0.1.0",
      type: "module",
      license: "AGPL-3.0-only",
      packageManager: "pnpm@10.33.0",
      publishConfig: { access: "public" },
    });
    expect(packageJson.engines.node).toBe(">=22");
    expect(packageJson.scripts["release:pack"]).toBe("node ./src/scripts/check-package.mjs --output .");
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
    const releaseScript = await text("src/scripts/publish-github-release.sh");
    const workflow = parse(source);

    expect(workflow.jobs.publish.permissions["id-token"]).toBe("write");
    expect(workflow.jobs.release.needs).toBe("publish");
    expect(source).toContain("node-version: 24");
    expect(source).toContain("registry-url: https://registry.npmjs.org");
    expect(source).toContain("npm install --global npm@latest");
    expect(source).toContain('npm publish "$TARBALL" --access public');
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
