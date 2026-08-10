#!/bin/sh
set -eu

repository=${STABILEO_REPOSITORY:-https://github.com/lambdaclass/stabileo.git}
revision=${STABILEO_REF:-main}
script_directory=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd -P)
project_root=$(CDPATH= cd -- "$script_directory/../.." && pwd -P)
vendored_directory="$project_root/vendored"

if [ ! -f "$project_root/package.json" ]; then
  echo "vendor-stabileo: expected package.json at $project_root" >&2
  exit 1
fi

if [ -L "$vendored_directory" ]; then
  echo "vendor-stabileo: refusing to replace symlink $vendored_directory" >&2
  exit 1
fi

temporary_base=${TMPDIR:-/tmp}
temporary_directory=$(mktemp -d "$temporary_base/stabileo-vendor.XXXXXX")
checkout_directory="$temporary_directory/repository"
staged_directory="$temporary_directory/vendored"
previous_directory="$temporary_directory/previous-vendored"
swap_started=0

cleanup() {
  status=$?
  trap - EXIT HUP INT TERM

  if [ "$swap_started" -eq 1 ] && [ ! -e "$vendored_directory" ] && [ -e "$previous_directory" ]; then
    mv -- "$previous_directory" "$vendored_directory" || true
  fi

  case "$temporary_directory" in
    "$temporary_base"/stabileo-vendor.*)
      rm -rf -- "$temporary_directory"
      ;;
    *)
      echo "vendor-stabileo: refusing to clean unexpected temporary path" >&2
      ;;
  esac

  exit "$status"
}
trap cleanup EXIT HUP INT TERM

git clone --quiet --filter=blob:none --no-checkout --sparse "$repository" "$checkout_directory"
git -C "$checkout_directory" sparse-checkout set engine
git -C "$checkout_directory" fetch --quiet --depth=1 origin "$revision"
git -C "$checkout_directory" checkout --quiet --detach FETCH_HEAD

if [ ! -f "$checkout_directory/engine/Cargo.toml" ] || [ ! -f "$checkout_directory/LICENSE" ]; then
  echo "vendor-stabileo: upstream checkout is missing engine/Cargo.toml or LICENSE" >&2
  exit 1
fi

mkdir -p "$staged_directory/stabileo"
cp -R "$checkout_directory/engine/." "$staged_directory/stabileo"
cp "$checkout_directory/LICENSE" "$staged_directory/stabileo/LICENSE"
git -C "$checkout_directory" rev-parse HEAD > "$staged_directory/STABILEO_REVISION"

mkdir -p "$project_root"
cp "$checkout_directory/LICENSE" "$temporary_directory/LICENSE"

if [ -e "$vendored_directory" ]; then
  mv -- "$vendored_directory" "$previous_directory"
  swap_started=1
fi

mv -- "$staged_directory" "$vendored_directory"
swap_started=0

cp "$temporary_directory/LICENSE" "$project_root/.LICENSE.next.$$"
mv -- "$project_root/.LICENSE.next.$$" "$project_root/LICENSE"

resolved_revision=$(sed -n '1p' "$vendored_directory/STABILEO_REVISION")
echo "Vendored Stabileo engine at $resolved_revision"
