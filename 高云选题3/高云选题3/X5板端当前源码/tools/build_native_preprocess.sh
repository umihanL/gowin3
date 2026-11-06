#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source_path="src/rescue_vision/rescue_vision/native_preprocess.cpp"
output_path="${1:-src/rescue_vision/rescue_vision/_native_preprocess.so}"
compiler="${CXX:-g++}"
if ! command -v "$compiler" >/dev/null 2>&1; then
  echo 'A C++ compiler (g++) is required.' >&2
  exit 1
fi
options=(-O3 -std=c++11 -shared -fPIC -fopenmp -ffp-contract=off -fno-math-errno)
if [ "$(uname -m)" = aarch64 ]; then
  options+=(-mcpu=cortex-a55)
fi
"$compiler" "${options[@]}" "$source_path" -o "$output_path"
echo "Built full-range float32 NV12 preprocessor: $output_path"
