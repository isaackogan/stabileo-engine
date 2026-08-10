import initializeWasm, { initSync as initializeWasmSync } from "./generated/wasm/stabileo_engine.js";
import * as wasmBindings from "./generated/wasm/stabileo_engine.js";
import {
  generatedOperationDefinitions,
  type GeneratedStabileoMethods,
} from "./generated/bindings.js";

export type WasmSource = BufferSource | WebAssembly.Module | Response | RequestInfo | URL;

export interface InitStabileoEngineOptions {
  wasm?: WasmSource | Promise<WasmSource>;
}

export interface InitStabileoEngineSyncOptions {
  wasm: BufferSource | WebAssembly.Module;
}

export type StabileoEngineErrorCode =
  | "INITIALIZATION"
  | "SERIALIZATION"
  | "JSON_PARSE"
  | "WASM";

export class StabileoEngineError extends Error {
  readonly code: StabileoEngineErrorCode;

  constructor(code: StabileoEngineErrorCode, message: string, options?: ErrorOptions) {
    super(message, options);
    this.name = "StabileoEngineError";
    this.code = code;
  }
}

type WasmBindings = typeof wasmBindings;
type RawOperationName = keyof typeof generatedOperationDefinitions;

export type RawStabileoEngine = Readonly<
  Pick<WasmBindings, Extract<RawOperationName, keyof WasmBindings>>
>;

export interface StabileoEngine extends GeneratedStabileoMethods {
  readonly raw: RawStabileoEngine;
}

let engineHandle: StabileoEngine | undefined;
let initializationPromise: Promise<StabileoEngine> | undefined;

function errorMessage(value: unknown) {
  if (value instanceof Error) return value.message;
  if (typeof value === "string") return value;
  try {
    return JSON.stringify(value);
  } catch {
    return String(value);
  }
}

function normalizedError(
  code: StabileoEngineErrorCode,
  context: string,
  cause: unknown,
) {
  if (cause instanceof StabileoEngineError) return cause;
  return new StabileoEngineError(code, `${context}: ${errorMessage(cause)}`, { cause });
}

function jsonInput(method: string, value: unknown) {
  try {
    const serialized = JSON.stringify(value);
    if (serialized === undefined) throw new TypeError("input is not JSON-serializable");
    return serialized;
  } catch (error) {
    throw normalizedError("SERIALIZATION", `${method} could not serialize its input`, error);
  }
}

function jsonOutput(method: string, value: unknown) {
  if (typeof value !== "string") return value;
  try {
    return JSON.parse(value) as unknown;
  } catch (error) {
    throw normalizedError("JSON_PARSE", `${method} returned invalid JSON`, error);
  }
}

function createEngine(): StabileoEngine {
  if (engineHandle) return engineHandle;

  const rawEntries = Object.keys(generatedOperationDefinitions).map((rawName) => [
    rawName,
    wasmBindings[rawName as keyof WasmBindings],
  ]);
  const raw = Object.freeze(Object.fromEntries(rawEntries)) as RawStabileoEngine;
  const engine = { raw } as StabileoEngine;

  for (const [rawName, definition] of Object.entries(generatedOperationDefinitions)) {
    Object.defineProperty(engine, definition.method, {
      enumerable: true,
      value: (...args: unknown[]) => {
        const rawFunction = raw[rawName as keyof RawStabileoEngine] as unknown as (...values: unknown[]) => unknown;
        const rawArguments = definition.input === null
          ? args
          : [definition.transport === "json" ? jsonInput(definition.method, args[0]) : args[0], ...args.slice(1)];
        let result;
        try {
          result = rawFunction(...rawArguments);
        } catch (error) {
          throw normalizedError("WASM", `${definition.method} failed`, error);
        }
        return definition.transport === "json" ? jsonOutput(definition.method, result) : result;
      },
    });
  }

  engineHandle = Object.freeze(engine);
  return engineHandle;
}

async function nodeWasmBytes(url: URL): Promise<Uint8Array> {
  const importNodeModule = Function(
    "specifier",
    "return import(specifier)",
  ) as (specifier: string) => Promise<typeof import("node:fs/promises")>;
  const { readFile } = await importNodeModule("node:fs/promises");
  return readFile(url) as Promise<Uint8Array>;
}

async function defaultWasmSource(): Promise<WasmSource> {
  const url = new URL("./stabileo-engine.wasm", import.meta.url);
  const nodeProcess = globalThis as typeof globalThis & {
    process?: { versions?: { node?: string } };
  };
  if (nodeProcess.process?.versions?.node) return nodeWasmBytes(url);

  let response: Response;
  try {
    response = await fetch(url);
  } catch (error) {
    throw normalizedError("INITIALIZATION", `could not fetch ${url.href}`, error);
  }
  if (!response.ok) {
    throw new StabileoEngineError(
      "INITIALIZATION",
      `could not fetch ${url.href}: HTTP ${response.status}`,
    );
  }
  return response;
}

export function initStabileoEngine(
  options: InitStabileoEngineOptions = {},
): Promise<StabileoEngine> {
  if (engineHandle) return Promise.resolve(engineHandle);
  if (initializationPromise) return initializationPromise;

  initializationPromise = (async () => {
    try {
      const source = await (options.wasm ?? defaultWasmSource());
      await initializeWasm({ module_or_path: source });
      return createEngine();
    } catch (error) {
      throw normalizedError("INITIALIZATION", "could not initialize Stabileo WebAssembly", error);
    }
  })();

  initializationPromise.catch(() => {
    initializationPromise = undefined;
  });
  return initializationPromise;
}

export function initStabileoEngineSync(
  options: InitStabileoEngineSyncOptions,
): StabileoEngine {
  if (engineHandle) return engineHandle;
  try {
    initializeWasmSync({ module: options.wasm });
    return createEngine();
  } catch (error) {
    throw normalizedError("INITIALIZATION", "could not initialize Stabileo WebAssembly", error);
  }
}
