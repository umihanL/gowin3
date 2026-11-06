// Fixed-contract hbDNN adapter. ctypes.CDLL releases the Python GIL around each
// call, including the BPU wait; no pixels, weights or logits are approximated.
#include <dnn/hb_dnn.h>
#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

namespace {
constexpr size_t kInputBytes = 640 * 640 * 3 / 2;
constexpr size_t kOutputFloats = (80 * 80 + 40 * 40 + 20 * 20) * (3 + 64);
const char* kNames[] = {"cls_s8", "box_s8", "cls_s16", "box_s16", "cls_s32", "box_s32"};

void check(int code, const char* call) {
  if (code) throw std::runtime_error(std::string(call) + " failed: " + std::to_string(code));
}
void error_text(char* out, size_t capacity, const char* text) {
  if (out && capacity) std::snprintf(out, capacity, "%s", text);
}

struct Context {
  hbPackedDNNHandle_t packed = nullptr;
  hbDNNHandle_t model = nullptr;
  hbDNNTaskHandle_t task = nullptr;
  hbDNNTensor input{};
  std::array<hbDNNTensor, 6> outputs{};
  std::array<int, 6> canonical_to_model{};
  bool failed = false;
  bool task_completed = false;
  bool release_uncertain = false;
  ~Context() {
    // Only called when there was no submitted task or finish_context confirmed
    // completion. An uncertain task keeps this ENTIRE context alive instead.
    for (auto& tensor : outputs)
      if (tensor.sysMem[0].virAddr) hbSysFreeMem(&tensor.sysMem[0]);
    if (input.sysMem[0].virAddr) hbSysFreeMem(&input.sysMem[0]);
    if (packed) hbDNNRelease(packed);
  }
};

bool finish_context(Context& c) noexcept {
  if (c.release_uncertain) return false;
  if (!c.task) return true;
  if (!c.task_completed) {
    if (hbDNNWaitTaskDone(c.task, 0) != 0) {
      // ReleaseTask cancels asynchronously. The tensor structures and all DMA
      // buffers must remain allocated even after it returns.
      hbDNNReleaseTask(c.task);
      c.release_uncertain = true;
      return false;
    }
    c.task_completed = true;
  }
  if (hbDNNReleaseTask(c.task) != 0) {
    c.release_uncertain = true;
    return false;
  }
  c.task = nullptr;
  return true;
}

bool equal_shape(const hbDNNTensorShape& a, const hbDNNTensorShape& b) {
  if (a.numDimensions != b.numDimensions) return false;
  for (int i = 0; i < a.numDimensions; ++i)
    if (a.dimensionSize[i] != b.dimensionSize[i]) return false;
  return true;
}

std::array<size_t, 4> output_strides(const hbDNNTensorProperties& p) {
  std::array<size_t, 4> strides{};
  bool explicit_strides = true;
  for (int i = 0; i < 4; ++i) explicit_strides &= p.stride[i] > 0;
  if (explicit_strides) {
    for (int i = 0; i < 4; ++i) strides[i] = p.stride[i];
  } else {
    // Older hbDNN versions leave stride[] zero. The documented aligned shape
    // supplies the physical dense tensor layout in that case.
    for (int i = 0; i < 4; ++i)
      if (p.stride[i] != 0) throw std::runtime_error("Partial output stride metadata unsupported");
    strides[3] = sizeof(float);
    for (int i = 2; i >= 0; --i)
      strides[i] = strides[i + 1] * p.alignedShape.dimensionSize[i + 1];
  }
  if (strides[3] != sizeof(float)) throw std::runtime_error("Output channel stride is not float32");
  size_t last = sizeof(float);
  for (int i = 0; i < 4; ++i) last += (p.validShape.dimensionSize[i] - 1) * strides[i];
  if (last > static_cast<size_t>(p.alignedByteSize))
    throw std::runtime_error("Output strides exceed allocated bytes");
  for (int i = 0; i < 3; ++i)
    if (strides[i] < strides[i + 1] * p.validShape.dimensionSize[i + 1])
      throw std::runtime_error("Overlapping output strides unsupported");
  return strides;
}

void validate_output(const hbDNNTensorProperties& p, int canonical) {
  int side = 80 >> (canonical / 2), channels = canonical % 2 ? 64 : 3;
  if (p.tensorType != HB_DNN_TENSOR_TYPE_F32 || p.quantiType != NONE)
    throw std::runtime_error("Only unquantized float32 model outputs are supported");
  // This mapper emits channel-last shapes/strides but tags float output tensors
  // as NCHW (2). Trust the exact named shape plus physical strides, as pyeasy
  // does; never transpose according to that generic layout tag.
  if ((p.tensorLayout != HB_DNN_LAYOUT_NHWC && p.tensorLayout != HB_DNN_LAYOUT_NCHW) || p.validShape.numDimensions != 4 ||
      p.alignedShape.numDimensions != 4 || p.validShape.dimensionSize[0] != 1 ||
      p.validShape.dimensionSize[1] != side || p.validShape.dimensionSize[2] != side ||
      p.validShape.dimensionSize[3] != channels || p.alignedByteSize <= 0)
    throw std::runtime_error("Output does not satisfy six-head NHWC YOLO11 contract: " +
        std::string(kNames[canonical]) + " layout=" + std::to_string(p.tensorLayout) +
        " rank=" + std::to_string(p.validShape.numDimensions) +
        " dims=" + std::to_string(p.validShape.dimensionSize[0]) + "," +
        std::to_string(p.validShape.dimensionSize[1]) + "," +
        std::to_string(p.validShape.dimensionSize[2]) + "," +
        std::to_string(p.validShape.dimensionSize[3]) + " bytes=" + std::to_string(p.alignedByteSize));
  for (int i = 0; i < 4; ++i)
    if (p.alignedShape.dimensionSize[i] < p.validShape.dimensionSize[i])
      throw std::runtime_error("Aligned shape smaller than valid shape");
  output_strides(p);
}

void initialize(Context& c, const char* path) {
  check(hbDNNInitializeFromFiles(&c.packed, &path, 1), "hbDNNInitializeFromFiles");
  const char** names = nullptr;
  int32_t count = 0;
  check(hbDNNGetModelNameList(&names, &count, c.packed), "hbDNNGetModelNameList");
  if (count != 1) throw std::runtime_error("Expected exactly one model in BIN");
  check(hbDNNGetModelHandle(&c.model, c.packed, names[0]), "hbDNNGetModelHandle");
  check(hbDNNGetInputCount(&count, c.model), "hbDNNGetInputCount");
  if (count != 1) throw std::runtime_error("Expected one packed NV12 input");
  auto& p = c.input.properties;
  check(hbDNNGetInputTensorProperties(&p, c.model, 0), "hbDNNGetInputTensorProperties");
  const bool nhwc = p.tensorLayout == HB_DNN_LAYOUT_NHWC &&
      p.validShape.dimensionSize[1] == 640 && p.validShape.dimensionSize[2] == 640 &&
      p.validShape.dimensionSize[3] == 3;
  const bool nchw = p.tensorLayout == HB_DNN_LAYOUT_NCHW &&
      p.validShape.dimensionSize[1] == 3 && p.validShape.dimensionSize[2] == 640 &&
      p.validShape.dimensionSize[3] == 640;
  if (p.tensorType != HB_DNN_IMG_TYPE_NV12 || p.validShape.numDimensions != 4 ||
      p.validShape.dimensionSize[0] != 1 || !(nhwc || nchw) ||
      !equal_shape(p.validShape, p.alignedShape) || p.alignedByteSize != kInputBytes)
    throw std::runtime_error("Input must be contiguous packed 640x640 NV12 without padding");
  check(hbSysAllocCachedMem(&c.input.sysMem[0], p.alignedByteSize), "allocate input");
  check(hbDNNGetOutputCount(&count, c.model), "hbDNNGetOutputCount");
  if (count != 6) throw std::runtime_error("Expected exactly six outputs");
  c.canonical_to_model.fill(-1);
  for (int i = 0; i < 6; ++i) {
    const char* name = nullptr;
    check(hbDNNGetOutputName(&name, c.model, i), "hbDNNGetOutputName");
    int canonical = -1;
    for (int j = 0; j < 6; ++j) if (std::strcmp(name, kNames[j]) == 0) canonical = j;
    if (canonical < 0 || c.canonical_to_model[canonical] >= 0)
      throw std::runtime_error("Unexpected/duplicate output name");
    c.canonical_to_model[canonical] = i;
    auto& out = c.outputs[i];
    check(hbDNNGetOutputTensorProperties(&out.properties, c.model, i), "hbDNNGetOutputTensorProperties");
    validate_output(out.properties, canonical);
    check(hbSysAllocCachedMem(&out.sysMem[0], out.properties.alignedByteSize), "allocate output");
  }
}
}  // namespace

extern "C" int rescue_dnn_version() { return 1; }

extern "C" void* rescue_dnn_create(const char* path, char* error, size_t capacity) {
  try {
    if (!path || !path[0]) throw std::runtime_error("Empty model path");
    std::unique_ptr<Context> c(new Context());
    initialize(*c, path);
    return c.release();
  } catch (const std::exception& ex) { error_text(error, capacity, ex.what()); }
  catch (...) { error_text(error, capacity, "Unknown native initialization failure"); }
  return nullptr;
}

extern "C" int rescue_dnn_forward(void* handle, const unsigned char* data,
    size_t input_bytes, float* destination, size_t output_floats,
    char* error, size_t capacity) {
  try {
    if (!handle || !data || !destination || input_bytes != kInputBytes || output_floats != kOutputFloats)
      throw std::runtime_error("Invalid native forward buffers");
    auto& c = *static_cast<Context*>(handle);
    if (c.failed || c.task) throw std::runtime_error("Native runtime failed; recreate before reuse");
    std::memcpy(c.input.sysMem[0].virAddr, data, kInputBytes);
    check(hbSysFlushMem(&c.input.sysMem[0], HB_SYS_MEM_CACHE_CLEAN), "clean input cache");
    hbDNNInferCtrlParam control{};
    HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&control);
    hbDNNTensor* outputs = c.outputs.data();
    const int infer_code = hbDNNInfer(&c.task, &outputs, &c.input, c.model, &control);
    if (infer_code && c.task) c.release_uncertain = true;
    check(infer_code, "hbDNNInfer");
    check(hbDNNWaitTaskDone(c.task, 0), "hbDNNWaitTaskDone");
    c.task_completed = true;
    const int release_code = hbDNNReleaseTask(c.task);
    if (release_code) c.release_uncertain = true;
    check(release_code, "hbDNNReleaseTask");
    c.task = nullptr;
    c.task_completed = false;
    size_t offset = 0;
    for (int canonical = 0; canonical < 6; ++canonical) {
      auto& out = c.outputs[c.canonical_to_model[canonical]];
      check(hbSysFlushMem(&out.sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE), "invalidate output cache");
      const auto strides = output_strides(out.properties);
      const int side = 80 >> (canonical / 2), channels = canonical % 2 ? 64 : 3;
      const auto* src = static_cast<const unsigned char*>(out.sysMem[0].virAddr);
      const size_t bytes = side * side * channels * sizeof(float);
      if (strides[2] == channels * sizeof(float) && strides[1] == side * strides[2]) {
        std::memcpy(destination + offset, src, bytes);
      } else {
        for (int y = 0; y < side; ++y)
          for (int x = 0; x < side; ++x)
            std::memcpy(destination + offset + (y * side + x) * channels,
                        src + y * strides[1] + x * strides[2], channels * sizeof(float));
      }
      offset += bytes / sizeof(float);
    }
    return 0;
  } catch (const std::exception& ex) {
    if (handle) static_cast<Context*>(handle)->failed = true;
    error_text(error, capacity, ex.what());
  } catch (...) {
    if (handle) static_cast<Context*>(handle)->failed = true;
    error_text(error, capacity, "Unknown native forward failure");
  }
  return -1;
}

extern "C" int rescue_dnn_destroy(void* handle) {
  auto* c = static_cast<Context*>(handle);
  if (!c) return 0;
  if (!finish_context(*c)) {
    c->failed = true;
    std::fprintf(stderr, "Native hbDNN cleanup: task completion/release unconfirmed; "
                         "quarantined context and DMA memory until process exit. Restart process.\n");
    return -1;
  }
  delete c;
  return 0;
}
