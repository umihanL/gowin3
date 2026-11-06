#!/usr/bin/env bash
# Build the pinned CPU runtime against this board's Ubuntu system libraries.
set -eo pipefail
cd "$(dirname "$0")/.."
edge_root="${RESCUE_EDGE_AI_ROOT:-/root/rescue/edge_ai}"
source_commit=aa94f2086147cd4bfbcff818ce998dffce5fcb39
archive="${RESCUE_LLAMA_SOURCE_ARCHIVE:-$edge_root/downloads/llama_cpp_source.tar.gz}"
expected=f2df238f09ecf19cf368b412fcbff86901f97ec867f1336fdf953cfd3e26930a
for executable in cmake g++ sha256sum tar; do
  command -v "$executable" >/dev/null || { echo "Missing build tool: $executable" >&2; exit 2; }
done
[ "$(uname -m)" = aarch64 ] || { echo 'Run this script on the X5 ARM64 board.' >&2; exit 2; }
[ -f "$archive" ] || { echo "Missing pinned source archive: $archive" >&2; exit 2; }
actual="$(sha256sum "$archive" | cut -d ' ' -f 1)"
[ "$actual" = "$expected" ] || { echo 'Runtime source SHA256 mismatch.' >&2; exit 2; }
mkdir -p "$edge_root/source"
source_dir="$edge_root/source/llama.cpp-$source_commit"
if [ ! -f "$source_dir/CMakeLists.txt" ]; then
  tar -xzf "$archive" -C "$edge_root/source" --no-same-owner
  # A board without a set RTC can otherwise rebuild everything forever.
  # Change only this extracted source tree's metadata, never the system clock.
  find "$source_dir" -type f -exec touch {} +
fi
cmake -S "$source_dir" -B "$edge_root/build" \
  -DCMAKE_BUILD_TYPE=Release -DGGML_NATIVE=ON -DLLAMA_CURL=OFF \
  -DLLAMA_BUILD_TESTS=OFF -DLLAMA_BUILD_EXAMPLES=OFF -DLLAMA_BUILD_SERVER=ON
nice -n 10 cmake --build "$edge_root/build" --target llama-server --parallel 2
"$edge_root/build/bin/llama-server" --version
