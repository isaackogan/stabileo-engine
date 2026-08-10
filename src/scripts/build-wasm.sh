#!/bin/sh
set -eu

script_directory=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd -P)
project_root=$(CDPATH= cd -- "$script_directory/../.." && pwd -P)
engine_directory="$project_root/vendored/stabileo"
revision_file="$project_root/vendored/STABILEO_REVISION"
wasm_destination="$project_root/vendored/stabileo-engine.wasm"
generated_directory="$project_root/src/generated/wasm"
rustup_bin=${RUSTUP_BIN:-rustup}
wasm_pack_bin=${WASM_PACK_BIN:-wasm-pack}

if [ ! -f "$project_root/package.json" ] || [ ! -f "$engine_directory/Cargo.toml" ]; then
  echo "build-wasm: run npm run vendor before building" >&2
  exit 1
fi

if [ ! -f "$revision_file" ]; then
  echo "build-wasm: missing vendored/STABILEO_REVISION" >&2
  exit 1
fi

if [ -L "$project_root/vendored" ] || [ -L "$generated_directory" ]; then
  echo "build-wasm: refusing to write through a symlinked output directory" >&2
  exit 1
fi

"$rustup_bin" run nightly rustc --version >/dev/null
if ! "$rustup_bin" target list --installed --toolchain nightly | grep -qx 'wasm32-unknown-unknown'; then
  echo "build-wasm: nightly is missing target wasm32-unknown-unknown" >&2
  exit 1
fi
if ! "$rustup_bin" component list --installed --toolchain nightly | grep -q '^rust-src'; then
  echo "build-wasm: nightly is missing component rust-src" >&2
  exit 1
fi

wasm_pack_version=$("$wasm_pack_bin" --version)
if [ "$wasm_pack_version" != "wasm-pack 0.13.1" ]; then
  echo "build-wasm: expected wasm-pack 0.13.1, got $wasm_pack_version" >&2
  exit 1
fi

source_revision=$(sed -n '1p' "$revision_file")
if [ -z "$source_revision" ]; then
  echo "build-wasm: vendored revision is empty" >&2
  exit 1
fi

temporary_base=${TMPDIR:-/tmp}
temporary_directory=$(mktemp -d "$temporary_base/stabileo-wasm-build.XXXXXX")
package_directory="$temporary_directory/package"

cleanup() {
  status=$?
  trap - EXIT HUP INT TERM
  case "$temporary_directory" in
    "$temporary_base"/stabileo-wasm-build.*)
      rm -rf -- "$temporary_directory"
      ;;
    *)
      echo "build-wasm: refusing to clean unexpected temporary path" >&2
      ;;
  esac
  exit "$status"
}
trap cleanup EXIT HUP INT TERM

DEDALIANO_BUILD_SHA="$source_revision" "$wasm_pack_bin" build "$engine_directory" \
  --target web \
  --release \
  --no-opt \
  --out-dir "$package_directory" \
  --out-name stabileo_engine \
  -- \
  --locked

for output in \
  "$package_directory/stabileo_engine_bg.wasm" \
  "$package_directory/stabileo_engine.js" \
  "$package_directory/stabileo_engine.d.ts"
do
  if [ ! -s "$output" ]; then
    echo "build-wasm: wasm-pack did not produce $output" >&2
    exit 1
  fi
done

mkdir -p "$project_root/vendored" "$generated_directory"
cp "$package_directory/stabileo_engine_bg.wasm" "$wasm_destination.next.$$"
cp "$package_directory/stabileo_engine.js" "$generated_directory/stabileo_engine.js.next.$$"
cp "$package_directory/stabileo_engine.d.ts" "$generated_directory/stabileo_engine.d.ts.next.$$"

mv -- "$wasm_destination.next.$$" "$wasm_destination"
mv -- "$generated_directory/stabileo_engine.js.next.$$" "$generated_directory/stabileo_engine.js"
mv -- "$generated_directory/stabileo_engine.d.ts.next.$$" "$generated_directory/stabileo_engine.d.ts"

echo "Built $wasm_destination from Stabileo $source_revision"
