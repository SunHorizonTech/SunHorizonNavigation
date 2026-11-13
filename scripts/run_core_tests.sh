#!/usr/bin/env bash
set -euo pipefail
project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
build_dir=$(mktemp -d "${TMPDIR:-/tmp}/sunhorizon-public-tests.XXXXXX")
trap 'rm -rf "$build_dir"' EXIT
cmake_args=()
if [[ "$(uname -s)" == Darwin ]]; then
  sdk_path=$(xcrun --show-sdk-path)
  cmake_args+=("-DCMAKE_OSX_SYSROOT=$sdk_path" "-DCMAKE_CXX_FLAGS=-isystem $sdk_path/usr/include/c++/v1")
fi
cmake -S "$project_root" -B "$build_dir" "${cmake_args[@]}"
cmake --build "$build_dir" --parallel 2
ctest --test-dir "$build_dir" --output-on-failure
