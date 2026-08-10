#!/bin/sh
set -eu

tag=${1:-}
version=${2:-}
tarball=${3:-}
wasm=${4:-}

case "$tag" in
  v[0-9]*) ;;
  *) echo "release: invalid tag: $tag" >&2; exit 1 ;;
esac

if [ "$tag" != "v$version" ] || [ ! -f "$tarball" ] || [ ! -f "$wasm" ]; then
  echo "release: expected <vVERSION> <VERSION> <tarball> <wasm>" >&2
  exit 1
fi
if [ "$(basename -- "$tarball")" != "stabileo-engine-$version.tgz" ]; then
  echo "release: unexpected tarball name" >&2
  exit 1
fi
if [ "$(basename -- "$wasm")" != "stabileo-engine.wasm" ]; then
  echo "release: unexpected WASM name" >&2
  exit 1
fi

git ls-remote --exit-code --tags origin "refs/tags/$tag" >/dev/null

if ! gh release view "$tag" >/dev/null 2>&1; then
  gh release create "$tag" "$tarball" "$wasm" \
    --verify-tag \
    --generate-notes \
    --title "stabileo-engine $tag"
  exit 0
fi

draft=$(gh release view "$tag" --json isDraft --jq '.isDraft')
temporary_base=${RUNNER_TEMP:-${TMPDIR:-/tmp}}
temporary_directory=$(mktemp -d "$temporary_base/stabileo-release.XXXXXX")

cleanup() {
  status=$?
  trap - EXIT HUP INT TERM
  case "$temporary_directory" in
    "$temporary_base"/stabileo-release.*) rm -rf -- "$temporary_directory" ;;
    *) echo "release: refusing to clean unexpected temporary path" >&2 ;;
  esac
  exit "$status"
}
trap cleanup EXIT HUP INT TERM

for asset in "$tarball" "$wasm"
do
  asset_name=$(basename -- "$asset")
  if gh release view "$tag" --json assets --jq '.assets[].name' | grep -Fxq "$asset_name"; then
    gh release download "$tag" --pattern "$asset_name" --dir "$temporary_directory"
    if ! cmp -s "$asset" "$temporary_directory/$asset_name"; then
      echo "release: existing asset $asset_name does not match" >&2
      exit 1
    fi
  elif [ "$draft" = "true" ]; then
    gh release upload "$tag" "$asset"
  else
    echo "release: completed release is missing $asset_name" >&2
    exit 1
  fi
done

if [ "$draft" = "true" ]; then
  gh release edit "$tag" --draft=false --title "stabileo-engine $tag"
fi
