#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
compiler="${CXX:-g++}"
source_path="src/rescue_vision/rescue_vision/native_runtime.cpp"
output_path="${1:-src/rescue_vision/rescue_vision/_native_runtime.so}"
if ! command -v "$compiler" >/dev/null 2>&1; then
  echo 'A C++ compiler (g++) is required.' >&2
  exit 1
fi
options=(-O3 -std=c++11 -shared -fPIC -Wall -Wextra)
if [ "$(uname -m)" = aarch64 ]; then
  options+=(-mcpu=cortex-a55)
fi
"$compiler" "${options[@]}" "$source_path" -ldnn -o "$output_path"
echo "Built GIL-releasing hbDNN runtime: $output_path"
