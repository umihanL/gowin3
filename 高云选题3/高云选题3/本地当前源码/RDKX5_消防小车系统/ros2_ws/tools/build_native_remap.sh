#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source_path="src/rescue_vision/rescue_vision/native_remap.cpp"
output_path="${1:-src/rescue_vision/rescue_vision/_native_remap.so}"
compiler="${CXX:-g++}"
options=(-O3 -std=c++11 -shared -fPIC -fopenmp)
if [ "$(uname -m)" = aarch64 ]; then
  options+=(-mcpu=cortex-a55)
fi
"$compiler" "${options[@]}" "$source_path" -o "$output_path"
echo "Built exact fixed-point BGR remapper: $output_path"
