import { expect, test } from "@playwright/test";

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
    version: "0.1.0",
  });
});
