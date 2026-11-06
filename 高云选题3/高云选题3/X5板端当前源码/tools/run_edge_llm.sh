#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
runtime_bin="${RESCUE_LLAMA_SERVER_BIN:-/root/rescue/edge_ai/build/bin/llama-server}"
model_file="${RESCUE_LLM_MODEL:-$PWD/models/llm/Qwen3-1.7B-Q4_0.gguf}"
model_alias="${RESCUE_LLM_ALIAS:-qwen3-1.7b}"
model_threads="${RESCUE_LLM_THREADS:-4}"
model_context="${RESCUE_LLM_CONTEXT:-2048}"
case "$model_threads" in 1|2|3|4|5|6) ;; *) echo 'RESCUE_LLM_THREADS must be 1..6' >&2; exit 2;; esac
case "$model_context" in 1024|1536|2048) ;; *) echo 'RESCUE_LLM_CONTEXT must be 1024,1536,2048' >&2; exit 2;; esac
if [ ! -x "$runtime_bin" ] || [ ! -f "$model_file" ]; then
  echo 'Missing local llama-server or Qwen GGUF. See docs/完整学习与验收手册.md.' >&2
  exit 2
fi
export OMP_NUM_THREADS="$model_threads" OMP_WAIT_POLICY=PASSIVE GOMP_SPINCOUNT=0
export OPENBLAS_NUM_THREADS=1
reasoning_options=()
case "$model_alias" in
  qwen3-*) reasoning_options=(--reasoning off --reasoning-budget 0 --chat-template-kwargs '{"enable_thinking":false}') ;;
esac
exec "$runtime_bin" \
  --model "$model_file" --alias "$model_alias" \
  --host 127.0.0.1 --port 8088 --n-gpu-layers 0 \
  --threads "$model_threads" --threads-batch "$model_threads" --parallel 1 --ctx-size "$model_context" \
  --batch-size 128 --ubatch-size 128 --poll 0 --poll-batch 0 \
  --cache-ram 64 --no-webui --metrics "${reasoning_options[@]}" "$@"
