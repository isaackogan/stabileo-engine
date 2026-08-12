import { expect, test } from "@playwright/test";
import { readFile } from "node:fs/promises";

const packageJson = JSON.parse(
  await readFile(new URL("../../package.json", import.meta.url), "utf8"),
);

test("loads the sibling WASM and runs an analysis in a browser", async ({ page }) => {
  await page.goto("/");

  const result = await page.evaluate(async () => {
    const bundlePath = "/dist/index.js";
    const sdk = await import(bundlePath);
    const engine = await sdk.initStabileoEngine();
    const section = engine.analyzeSection({
      polygons: [{ vertices: [[0, 0], [2, 0], [2, 3], [0, 3]] }],
    });
    return {
      area: section.a,
      centroid: [section.yc, section.zc],
      hasRawSolver: typeof engine.raw.solve_2d === "function",
      version: sdk.version,
    };
  });

  expect(result).toEqual({
    area: 6,
    centroid: [1, 1.5],
    hasRawSolver: true,
    version: packageJson.version,
  });
});
