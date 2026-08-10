import { afterEach, describe, expect, test } from "vitest";
import { cp, mkdir, mkdtemp, readFile, rm, writeFile } from "node:fs/promises";
import { execFile } from "node:child_process";
import { promisify } from "node:util";
import { tmpdir } from "node:os";
import path from "node:path";

const execFileAsync = promisify(execFile);
const temporaryDirectories: string[] = [];

async function temporaryDirectory(prefix: string): Promise<string> {
  const directory = await mkdtemp(path.join(tmpdir(), prefix));
  temporaryDirectories.push(directory);
  return directory;
}

async function run(command: string, args: string[], cwd: string) {
  return execFileAsync(command, args, { cwd, encoding: "utf8" });
}

async function createUpstream(): Promise<{ root: string; revision: string }> {
  const root = await temporaryDirectory("stabileo-upstream-");
  await mkdir(path.join(root, "engine", ".cargo"), { recursive: true });
  await writeFile(path.join(root, "engine", "Cargo.toml"), "[package]\nname = \"fixture-engine\"\nversion = \"0.1.0\"\n");
  await writeFile(path.join(root, "engine", "solver.rs"), "pub const REVISION: u8 = 1;\n");
  await writeFile(path.join(root, "engine", ".cargo", "config.toml"), "[build]\n");
  await writeFile(path.join(root, "README.md"), "must not be vendored\n");
  await writeFile(path.join(root, "LICENSE"), "fixture AGPL license\n");
  await run("git", ["init", "-b", "main"], root);
  await run("git", ["config", "user.name", "Test"], root);
  await run("git", ["config", "user.email", "test@example.com"], root);
  await run("git", ["add", "."], root);
  await run("git", ["commit", "-m", "initial"], root);
  const { stdout } = await run("git", ["rev-parse", "HEAD"], root);
  return { root, revision: stdout.trim() };
}

async function createProject(): Promise<{ root: string; script: string }> {
  const root = await temporaryDirectory("stabileo-project-");
  const scripts = path.join(root, "src", "scripts");
  await mkdir(scripts, { recursive: true });
  await writeFile(path.join(root, "package.json"), "{}\n");
  const sourceScript = path.resolve("src/scripts/vendor-stabileo.sh");
  const script = path.join(scripts, "vendor-stabileo.sh");
  await cp(sourceScript, script);
  return { root, script };
}

afterEach(async () => {
  await Promise.all(temporaryDirectories.splice(0).map((directory) => rm(directory, { recursive: true, force: true })));
});
describe("vendor-stabileo.sh", () => {
  test("copies only engine plus licensing and revision provenance", async () => {
    const upstream = await createUpstream();
    const project = await createProject();

    await execFileAsync("sh", [project.script], {
      cwd: project.root,
      env: {
        ...process.env,
        STABILEO_REPOSITORY: upstream.root,
        STABILEO_REF: "main",
      },
    });

    expect(await readFile(path.join(project.root, "vendored", "stabileo", "solver.rs"), "utf8"))
      .toBe("pub const REVISION: u8 = 1;\n");
    expect(await readFile(path.join(project.root, "vendored", "stabileo", ".cargo", "config.toml"), "utf8"))
      .toBe("[build]\n");
    expect(await readFile(path.join(project.root, "vendored", "stabileo", "LICENSE"), "utf8"))
      .toBe("fixture AGPL license\n");
    expect(await readFile(path.join(project.root, "LICENSE"), "utf8"))
      .toBe("fixture AGPL license\n");
    expect(await readFile(path.join(project.root, "vendored", "STABILEO_REVISION"), "utf8"))
      .toBe(`${upstream.revision}\n`);
    await expect(readFile(path.join(project.root, "vendored", "stabileo", "README.md"), "utf8"))
      .rejects.toMatchObject({ code: "ENOENT" });
  });

  test("replaces an old snapshot but preserves it when acquisition fails", async () => {
    const upstream = await createUpstream();
    const project = await createProject();
    const destination = path.join(project.root, "vendored", "stabileo");
    await mkdir(destination, { recursive: true });
    await writeFile(path.join(destination, "old.txt"), "old snapshot\n");

    await execFileAsync("sh", [project.script], {
      cwd: project.root,
      env: { ...process.env, STABILEO_REPOSITORY: upstream.root },
    });
    await expect(readFile(path.join(destination, "old.txt"), "utf8"))
      .rejects.toMatchObject({ code: "ENOENT" });

    await expect(execFileAsync("sh", [project.script], {
      cwd: project.root,
      env: { ...process.env, STABILEO_REPOSITORY: path.join(upstream.root, "missing") },
    })).rejects.toBeDefined();
    expect(await readFile(path.join(destination, "solver.rs"), "utf8"))
      .toBe("pub const REVISION: u8 = 1;\n");
  });
});
