import { defineConfig } from "@playwright/test";

export default defineConfig({
  testDir: "./test/browser",
  fullyParallel: false,
  use: {
    baseURL: "http://127.0.0.1:4173",
  },
  webServer: {
    command: "node ./test/browser/server.mjs",
    url: "http://127.0.0.1:4173",
    reuseExistingServer: false,
  },
});
