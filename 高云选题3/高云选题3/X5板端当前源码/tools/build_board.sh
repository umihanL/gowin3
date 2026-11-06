#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source ./env.sh
if command -v g++ >/dev/null 2>&1; then
  bash tools/build_native_preprocess.sh
  bash tools/build_native_remap.sh
  if [ -f /usr/include/dnn/hb_dnn.h ]; then
    bash tools/build_native_runtime.sh
  else
    echo 'hbDNN development headers unavailable: existing pyeasy backend remains available.'
  fi
else
  echo 'g++ unavailable: using the compatible NumPy preprocessing fallback.'
fi
# Copy installation is intentional: setup.py packages workspace web/config assets.
# rescue_vision/setup.cfg forces build/install copies, including native libraries
# and share assets, even when the board RTC predates the existing installed files.
# Limiting compilation also leaves memory available for VS Code on the X5.
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
python -m colcon build --base-paths src --executor sequential \
  --cmake-args -DPython3_EXECUTABLE="$VIRTUAL_ENV/bin/python"
echo 'Build complete. Run: source ./env.sh'
