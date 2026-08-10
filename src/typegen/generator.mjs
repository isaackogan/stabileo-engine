import { createHash } from "node:crypto";

const NUMERIC_PRIMITIVES = new Set([
  "i8", "i16", "i32", "i64", "i128", "isize",
  "u8", "u16", "u32", "u64", "u128", "usize",
  "f16", "f32", "f64",
]);

const BUILTIN_TYPES = new Set([
  "boolean", "number", "string", "void", "undefined", "null", "unknown", "JsonValue",
]);

function itemAttributes(item) {
  return (item?.attrs ?? []).map((attribute) =>
    typeof attribute === "string" ? attribute : attribute?.other ?? "",
  );
}

function serdeOptions(item) {
  const options = {};
  for (const attribute of itemAttributes(item)) {
    const match = attribute.match(/^#\[serde\((.*)\)\]$/s);
    if (!match) continue;
    for (const fragment of match[1].split(/\s*,\s*/)) {
      const pair = fragment.match(/^([a-zA-Z_][\w]*)\s*=\s*"([^"]*)"$/);
      if (pair) options[pair[1]] = pair[2];
      else if (/^[a-zA-Z_][\w]*$/.test(fragment)) options[fragment] = true;
    }
  }
  return options;
}

function words(value) {
  return value
    .replace(/([a-z\d])([A-Z])/g, "$1 $2")
    .replace(/[^a-zA-Z\d]+/g, " ")
    .trim()
    .split(/\s+/)
    .filter(Boolean);
}

function rename(value, rule) {
  if (!rule) return value;
  const parts = words(value);
  switch (rule) {
    case "camelCase":
      return parts.map((part, index) => index === 0
        ? part.toLowerCase()
        : part[0].toUpperCase() + part.slice(1).toLowerCase()).join("");
    case "PascalCase":
      return parts.map((part) => part[0].toUpperCase() + part.slice(1).toLowerCase()).join("");
    case "snake_case":
      return parts.map((part) => part.toLowerCase()).join("_");
    case "SCREAMING_SNAKE_CASE":
      return parts.map((part) => part.toUpperCase()).join("_");
    case "kebab-case":
      return parts.map((part) => part.toLowerCase()).join("-");
    case "SCREAMING-KEBAB-CASE":
      return parts.map((part) => part.toUpperCase()).join("-");
    case "lowercase":
      return parts.join("").toLowerCase();
    case "UPPERCASE":
      return parts.join("").toUpperCase();
    default:
      throw new Error(`unsupported serde rename_all rule: ${rule}`);
  }
}

function genericTypeArguments(resolvedPath) {
  const args = resolvedPath?.args?.angle_bracketed?.args ?? [];
  return args.flatMap((argument) => argument && "type" in argument ? [argument.type] : []);
}

function shortPath(path) {
  return path.split("::").at(-1);
}

function quoteProperty(name) {
  return /^[A-Za-z_$][\w$]*$/.test(name) ? name : JSON.stringify(name);
}

function isOption(type) {
  return shortPath(type?.resolved_path?.path ?? "") === "Option";
}

export function wasmExports(declarations) {
  const exports = [];
  const expression = /export\s+function\s+([A-Za-z_$][\w$]*)\s*\(/g;
  for (const match of declarations.matchAll(expression)) {
    if (match[1] !== "initSync") exports.push(match[1]);
  }
  return [...new Set(exports)].sort();
}

function operationType(type) {
  if (type === null || type === undefined) return null;
  if (typeof type !== "string" || type.length === 0) {
    throw new Error("operation input/output types must be non-empty strings or null");
  }
  return type;
}

export function generateBindings(rustdoc, declarations, manifest) {
  if (!manifest || typeof manifest.operations !== "object" || Array.isArray(manifest.operations)) {
    throw new Error("manifest.operations must be an object");
  }

  const actualExports = wasmExports(declarations);
  const mappedExports = Object.keys(manifest.operations).sort();
  const missing = actualExports.filter((name) => !(name in manifest.operations));
  const extra = mappedExports.filter((name) => !actualExports.includes(name));
  if (missing.length) throw new Error(`missing manifest entries: ${missing.join(", ")}`);
  if (extra.length) throw new Error(`manifest entries without wasm exports: ${extra.join(", ")}`);

  const index = rustdoc?.index ?? {};
  const itemsByName = new Map();
  for (const item of Object.values(index)) {
    if (!item?.name) continue;
    const candidates = itemsByName.get(item.name) ?? [];
    candidates.push(item);
    itemsByName.set(item.name, candidates);
  }

  const pending = [];
  const queued = new Set();
  const declarationsByName = new Map();
  let typeContext = "unknown model";

  function queueNamedType(name) {
    if (BUILTIN_TYPES.has(name) || queued.has(name)) return;
    const candidates = itemsByName.get(name) ?? [];
    const eligible = candidates.filter((item) => item.inner?.struct || item.inner?.enum || item.inner?.type_alias);
    if (eligible.length !== 1) {
      throw new Error(eligible.length === 0
        ? `rustdoc type not found: ${name}`
        : `rustdoc type is ambiguous: ${name}`);
    }
    queued.add(name);
    pending.push(eligible[0]);
  }

  function translateType(type) {
    if (!type || typeof type !== "object") {
      throw new Error(`unsupported rustdoc type in ${typeContext}: missing type object`);
    }
    if (type.primitive) {
      if (NUMERIC_PRIMITIVES.has(type.primitive)) return "number";
      if (type.primitive === "bool") return "boolean";
      if (type.primitive === "str" || type.primitive === "char") return "string";
      if (type.primitive === "unit") return "void";
      throw new Error(`unsupported rustdoc type: primitive ${type.primitive}`);
    }
    if (type.resolved_path) {
      const path = shortPath(type.resolved_path.path);
      const args = genericTypeArguments(type.resolved_path);
      if (["String", "Cow"].includes(path)) return "string";
      if (path === "Option") {
        if (args.length !== 1) throw new Error("unsupported rustdoc type: Option arity");
        return `${translateType(args[0])} | null`;
      }
      if (["Vec", "VecDeque", "HashSet", "BTreeSet"].includes(path)) {
        if (args.length !== 1) throw new Error(`unsupported rustdoc type: ${path} arity`);
        return `Array<${translateType(args[0])}>`;
      }
      if (["HashMap", "BTreeMap", "IndexMap"].includes(path)) {
        if (args.length !== 2) throw new Error(`unsupported rustdoc type: ${path} arity`);
        const key = translateType(args[0]);
        if (key !== "string" && key !== "number") {
          throw new Error(`unsupported rustdoc type: ${path} key ${key}`);
        }
        return `Record<${key}, ${translateType(args[1])}>`;
      }
      if (["Box", "Rc", "Arc", "Cell", "RefCell"].includes(path)) {
        if (args.length !== 1) throw new Error(`unsupported rustdoc type: ${path} arity`);
        return translateType(args[0]);
      }
      if (["Result"].includes(path)) {
        throw new Error("unsupported rustdoc type: Result is not a serialized data model");
      }
      queueNamedType(path);
      return path;
    }
    if (type.borrowed_ref) return translateType(type.borrowed_ref.type);
    if (type.raw_pointer) return translateType(type.raw_pointer.type);
    if (type.slice) return `Array<${translateType(type.slice)}>`;
    if (type.array) return `Array<${translateType(type.array.type)}>`;
    if (type.tuple) {
      if (type.tuple.length === 0) return "void";
      return `[${type.tuple.map(translateType).join(", ")}]`;
    }
    if (type.generic) throw new Error(`unsupported rustdoc type: unresolved generic ${type.generic}`);
    if (type.infer) throw new Error("unsupported rustdoc type: inferred type");
    if (type.dyn_trait) throw new Error("unsupported rustdoc type: dyn trait");
    if (type.function_pointer) throw new Error("unsupported rustdoc type: function pointer");
    throw new Error(`unsupported rustdoc type: ${Object.keys(type).join(", ") || "unknown"}`);
  }

  function emitFields(fieldIds, containerOptions = {}, containerName = "anonymous struct") {
    return fieldIds.map((fieldId) => {
      const field = index[String(fieldId)] ?? index[fieldId];
      if (!field?.inner?.struct_field || !field.name) {
        throw new Error(`unsupported rustdoc struct field: ${fieldId}`);
      }
      const fieldOptions = serdeOptions(field);
      if (fieldOptions.skip || fieldOptions.skip_deserializing) return null;
      const name = fieldOptions.rename ?? rename(field.name, containerOptions.rename_all);
      const optional = isOption(field.inner.struct_field)
        || fieldOptions.default
        || fieldOptions.skip_serializing_if;
      const previousContext = typeContext;
      typeContext = `${containerName}.${field.name}`;
      try {
        return `  ${quoteProperty(name)}${optional ? "?" : ""}: ${translateType(field.inner.struct_field)};`;
      } finally {
        typeContext = previousContext;
      }
    }).filter(Boolean);
  }

  function emitStruct(item) {
    const details = item.inner.struct;
    const options = serdeOptions(item);
    if (details.kind?.plain) {
      if (details.kind.plain.has_stripped_fields) {
        throw new Error(`unsupported rustdoc type: stripped fields in ${item.name}`);
      }
      return [
        `export interface ${item.name} {`,
        ...emitFields(details.kind.plain.fields, options, item.name),
        "}",
      ].join("\n");
    }
    if (details.kind?.tuple) {
      const fields = details.kind.tuple.map((fieldId) => {
        if (fieldId === null) throw new Error(`unsupported rustdoc type: private tuple field in ${item.name}`);
        const field = index[String(fieldId)] ?? index[fieldId];
        return translateType(field.inner.struct_field);
      });
      return `export type ${item.name} = [${fields.join(", ")}];`;
    }
    if (details.kind?.unit) return `export type ${item.name} = Record<string, never>;`;
    throw new Error(`unsupported rustdoc type: struct kind in ${item.name}`);
  }

  function emitVariantPayload(variant, enumOptions, variantOptions) {
    const kind = variant.inner.variant.kind;
    const variantName = variantOptions.rename ?? rename(variant.name, enumOptions.rename_all);
    const tag = enumOptions.tag;
    const content = enumOptions.content;
    const tagProperty = tag ? `${quoteProperty(tag)}: ${JSON.stringify(variantName)}` : null;

    if (kind === "plain" || kind.unit) {
      return tagProperty ? `{ ${tagProperty} }` : JSON.stringify(variantName);
    }
    if (kind.tuple) {
      const values = kind.tuple.map((fieldOrType) => {
        const field = typeof fieldOrType === "number" || typeof fieldOrType === "string"
          ? index[String(fieldOrType)]
          : null;
        const resolvedType = field?.inner?.struct_field ?? fieldOrType;
        const previousContext = typeContext;
        typeContext = `${variant.name}.${field?.name ?? "tuple field"}`;
        try {
          return translateType(resolvedType);
        } finally {
          typeContext = previousContext;
        }
      });
      const payload = values.length === 1 ? values[0] : `[${values.join(", ")}]`;
      if (tagProperty && content) return `{ ${tagProperty}; ${quoteProperty(content)}: ${payload} }`;
      if (tagProperty) return `{ ${tagProperty}; value: ${payload} }`;
      return `{ ${JSON.stringify(variantName)}: ${payload} }`;
    }
    if (kind.struct) {
      const fields = emitFields(kind.struct.fields, variantOptions, variant.name).map((line) => line.trim());
      if (tagProperty && content) return `{ ${tagProperty}; ${quoteProperty(content)}: { ${fields.join(" ")} } }`;
      if (tagProperty) return `{ ${tagProperty}; ${fields.join(" ")} }`;
      return `{ ${JSON.stringify(variantName)}: { ${fields.join(" ")} } }`;
    }
    throw new Error(`unsupported rustdoc type: enum variant ${variant.name}`);
  }

  function emitEnum(item) {
    if (item.inner.enum.has_stripped_variants) {
      throw new Error(`unsupported rustdoc type: stripped variants in ${item.name}`);
    }
    const options = serdeOptions(item);
    const variants = item.inner.enum.variants.map((variantId) => {
      const variant = index[String(variantId)] ?? index[variantId];
      if (!variant?.inner?.variant) throw new Error(`unsupported rustdoc enum variant: ${variantId}`);
      return emitVariantPayload(variant, options, serdeOptions(variant));
    });
    return `export type ${item.name} = ${variants.join(" | ")};`;
  }

  for (const operation of Object.values(manifest.operations)) {
    const input = operationType(operation.input);
    const output = operationType(operation.output);
    if (operation.roots) {
      if (!Array.isArray(operation.roots)) throw new Error("operation roots must be an array");
      for (const root of operation.roots) queueNamedType(root);
    } else {
      if (input && /^[A-Z][A-Za-z\d_]*$/.test(input)) queueNamedType(input);
      if (output && /^[A-Z][A-Za-z\d_]*$/.test(output)) queueNamedType(output);
    }
  }

  while (pending.length) {
    const item = pending.shift();
    let declaration;
    if (item.inner.struct) declaration = emitStruct(item);
    else if (item.inner.enum) declaration = emitEnum(item);
    else if (item.inner.type_alias) {
      declaration = `export type ${item.name} = ${translateType(item.inner.type_alias.type)};`;
    } else {
      throw new Error(`unsupported rustdoc item: ${item.name}`);
    }
    declarationsByName.set(item.name, declaration);
  }

  const operationLines = mappedExports.map((rawName) => {
    const operation = manifest.operations[rawName];
    if (!operation.method || !["json", "value"].includes(operation.transport)) {
      throw new Error(`invalid operation manifest entry: ${rawName}`);
    }
    const input = operationType(operation.input);
    const output = operationType(operation.output) ?? "void";
    const parameters = operation.parameters ?? [];
    const args = [];
    if (input) args.push(`input: ${input}`);
    for (const parameter of parameters) {
      if (!parameter.name || !parameter.raw || typeof parameter.type !== "string") {
        throw new Error(`invalid scalar parameter for operation: ${rawName}`);
      }
      args.push(`${parameter.name}: ${parameter.type}`);
    }
    return `  ${operation.method}(${args.join(", ")}): ${output};`;
  });

  const normalizedOperations = Object.fromEntries(mappedExports.map((rawName) => {
    const operation = manifest.operations[rawName];
    return [rawName, {
      method: operation.method,
      transport: operation.transport,
      input: operation.input ?? null,
      output: operation.output ?? null,
      parameters: operation.parameters ?? [],
      roots: operation.roots ?? [],
    }];
  }));

  const source = [
    "// Generated by src/typegen/cli.mjs. Do not edit directly.",
    "",
    "export type JsonValue = null | boolean | number | string | JsonValue[] | { [key: string]: JsonValue };",
    "",
    ...[...declarationsByName.keys()].sort().flatMap((name) => [declarationsByName.get(name), ""]),
    "export interface GeneratedStabileoMethods {",
    ...operationLines,
    "}",
    "",
    `export const generatedOperationDefinitions = ${JSON.stringify(normalizedOperations, null, 2)} as const;`,
    "",
  ].join("\n");

  return {
    source,
    semantic: {
      rustdocFormatVersion: rustdoc.format_version,
      operations: normalizedOperations,
      sourceSha256: createHash("sha256").update(source).digest("hex"),
      wasmExports: actualExports,
    },
  };
}

export function bindingsLock(semantic) {
  const canonical = `${JSON.stringify(semantic, Object.keys(semantic).sort())}\n`;
  return {
    formatVersion: 1,
    semanticSha256: createHash("sha256").update(canonical).digest("hex"),
    ...semantic,
  };
}
