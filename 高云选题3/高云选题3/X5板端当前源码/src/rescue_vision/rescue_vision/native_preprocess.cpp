// BT.601 full-range NV12 with the model's top-left chroma sampling contract.
// Compile with -ffp-contract=off, without -ffast-math: preserve float32 rounding.
#include <cstddef>
#include <cstdint>
#include <limits>
#include <cstring>
#if defined(__aarch64__)
#include <arm_neon.h>
#endif

static inline uint8_t saturate_truncate(float value) {
    value = value < 0.0f ? 0.0f : value;
    value = value > 255.0f ? 255.0f : value;
    return static_cast<uint8_t>(value);
}

extern "C" int rescue_nv12_version() { return 1; }

extern "C" int rescue_bgr_to_nv12(
        const uint8_t* __restrict__ source, std::size_t height,
        std::size_t width, std::size_t stride,
        uint8_t* __restrict__ destination, std::size_t destination_size) {
    if (!source || !destination || !width || !height || (width & 1) || (height & 1)) return 1;
    if (width > std::numeric_limits<std::size_t>::max() / 3 || stride < width * 3) return 2;
    if (height > std::numeric_limits<std::size_t>::max() / width) return 2;
    const std::size_t pixels = width * height;
    if (pixels > std::numeric_limits<std::size_t>::max() - pixels / 2 ||
        destination_size < pixels + pixels / 2) return 3;

    for (std::size_t row = 0; row < height; ++row) {
        const uint8_t* __restrict__ input = source + row * stride;
        uint8_t* __restrict__ output = destination + row * width;
        for (std::size_t col = 0; col < width; ++col) {
            const float b = input[3 * col], g = input[3 * col + 1], r = input[3 * col + 2];
            output[col] = saturate_truncate((0.299f * r + 0.587f * g) + 0.114f * b);
        }
    }
    for (std::size_t row = 0; row < height; row += 2) {
        const uint8_t* __restrict__ input = source + row * stride;
        uint8_t* __restrict__ output = destination + pixels + (row / 2) * width;
        for (std::size_t col = 0; col < width; col += 2) {
            const float b = input[3 * col], g = input[3 * col + 1], r = input[3 * col + 2];
            const float u = ((-0.169f * r - 0.331f * g) + 0.5f * b) + 128.0f;
            const float v = ((0.5f * r - 0.419f * g) - 0.081f * b) + 128.0f;
            output[col] = saturate_truncate(u);
            output[col + 1] = saturate_truncate(v);
        }
    }
    return 0;
}

// The HD path has exactly a 2:1 resize and 48 gray rows above and below it.
// OpenCV's uint8 INTER_LINEAR special case at this ratio uses the same rounded
// four-pixel average: (p00 + p01 + p10 + p11 + 2) >> 2. Keep this dedicated
// symbol optional so version-1 libraries and other image sizes remain usable.
#if defined(__aarch64__)
static inline float32x4_t low_float(uint8x8_t values) {
    return vcvtq_f32_u32(vmovl_u16(vget_low_u16(vmovl_u8(values))));
}

static inline float32x4_t high_float(uint8x8_t values) {
    return vcvtq_f32_u32(vmovl_u16(vget_high_u16(vmovl_u8(values))));
}

static inline uint16x4_t clipped_truncated(float32x4_t values) {
    values = vmaxq_f32(values, vdupq_n_f32(0.0f));
    values = vminq_f32(values, vdupq_n_f32(255.0f));
    return vmovn_u32(vcvtq_u32_f32(values));
}

static inline float32x4_t luminance(float32x4_t b, float32x4_t g, float32x4_t r) {
    return vaddq_f32(vaddq_f32(vmulq_n_f32(r, .299f), vmulq_n_f32(g, .587f)),
                     vmulq_n_f32(b, .114f));
}
#endif

static int prepare_hd_nv12(
        const uint8_t* __restrict__ source, std::size_t height,
        std::size_t width, std::size_t stride,
        uint8_t* __restrict__ destination, std::size_t destination_size, int workers) {
    if (!source || !destination || height != 1088 || width != 1280 || stride < width * 3) return 1;
    if (workers < 1 || workers > 8) return 4;
    constexpr std::size_t output_width = 640, output_height = 640, top = 48;
    constexpr std::size_t pixels = output_width * output_height;
    if (destination_size < pixels + pixels / 2) return 3;
    // Evaluate the gray color with the same float32 operations as every other
    // pixel; hardcoding 114 is unsafe if conversion rounding is ever changed.
    const uint8_t gray_y = saturate_truncate((.299f * 114.0f + .587f * 114.0f) + .114f * 114.0f);
    const uint8_t gray_u = saturate_truncate(((-.169f * 114.0f - .331f * 114.0f) + .5f * 114.0f) + 128.0f);
    const uint8_t gray_v = saturate_truncate(((.5f * 114.0f - .419f * 114.0f) - .081f * 114.0f) + 128.0f);
    std::memset(destination, gray_y, top * output_width);
    std::memset(destination + (output_height - top) * output_width, gray_y, top * output_width);
    for (std::size_t row = 0; row < top / 2; ++row) {
        uint8_t* upper = destination + pixels + row * output_width;
        uint8_t* lower = destination + pixels + (output_height / 2 - top / 2 + row) * output_width;
        for (std::size_t col = 0; col < output_width; col += 2) {
            upper[col] = lower[col] = gray_u;
            upper[col + 1] = lower[col + 1] = gray_v;
        }
    }
#if defined(_OPENMP)
#pragma omp parallel for num_threads(workers) schedule(static) if(workers > 1)
#endif
    for (std::size_t row = 0; row < height / 2; ++row) {
        const uint8_t* upper = source + 2 * row * stride;
        const uint8_t* lower = upper + stride;
        uint8_t* output = destination + (row + top) * output_width;
        uint8_t* chroma = destination + pixels + ((row + top) / 2) * output_width;
#if defined(__aarch64__)
        for (std::size_t col = 0; col < output_width; col += 8) {
            const uint8x16x3_t a = vld3q_u8(upper + col * 6);
            const uint8x16x3_t b = vld3q_u8(lower + col * 6);
            const uint8x8_t blue = vrshrn_n_u16(vaddq_u16(vpaddlq_u8(a.val[0]), vpaddlq_u8(b.val[0])), 2);
            const uint8x8_t green = vrshrn_n_u16(vaddq_u16(vpaddlq_u8(a.val[1]), vpaddlq_u8(b.val[1])), 2);
            const uint8x8_t red = vrshrn_n_u16(vaddq_u16(vpaddlq_u8(a.val[2]), vpaddlq_u8(b.val[2])), 2);
            const uint16x4_t y0 = clipped_truncated(luminance(low_float(blue), low_float(green), low_float(red)));
            const uint16x4_t y1 = clipped_truncated(luminance(high_float(blue), high_float(green), high_float(red)));
            vst1_u8(output + col, vmovn_u16(vcombine_u16(y0, y1)));
            if ((row & 1) == 0) {
                const float32x4_t bf = low_float(vuzp_u8(blue, blue).val[0]);
                const float32x4_t gf = low_float(vuzp_u8(green, green).val[0]);
                const float32x4_t rf = low_float(vuzp_u8(red, red).val[0]);
                const float32x4_t uf = vaddq_f32(vaddq_f32(vsubq_f32(vmulq_n_f32(rf, -.169f),
                    vmulq_n_f32(gf, .331f)), vmulq_n_f32(bf, .5f)), vdupq_n_f32(128.0f));
                const float32x4_t vf = vaddq_f32(vsubq_f32(vsubq_f32(vmulq_n_f32(rf, .5f),
                    vmulq_n_f32(gf, .419f)), vmulq_n_f32(bf, .081f)), vdupq_n_f32(128.0f));
                const uint16x4_t u16 = clipped_truncated(uf), v16 = clipped_truncated(vf);
                const uint8x8_t u8 = vmovn_u16(vcombine_u16(u16, u16));
                const uint8x8_t v8 = vmovn_u16(vcombine_u16(v16, v16));
                vst1_u8(chroma + col, vzip_u8(u8, v8).val[0]);
            }
        }
#else
        for (std::size_t col = 0; col < output_width; ++col) {
            const std::size_t offset = col * 6;
            const float b = (upper[offset] + upper[offset + 3] + lower[offset] + lower[offset + 3] + 2) >> 2;
            const float g = (upper[offset + 1] + upper[offset + 4] + lower[offset + 1] + lower[offset + 4] + 2) >> 2;
            const float r = (upper[offset + 2] + upper[offset + 5] + lower[offset + 2] + lower[offset + 5] + 2) >> 2;
            output[col] = saturate_truncate((.299f * r + .587f * g) + .114f * b);
            if (((row | col) & 1) == 0) {
                chroma[col] = saturate_truncate(((-.169f * r - .331f * g) + .5f * b) + 128.0f);
                chroma[col + 1] = saturate_truncate(((.5f * r - .419f * g) - .081f * b) + 128.0f);
            }
        }
#endif
    }
    return 0;
}

extern "C" int rescue_prepare_hd_nv12(
        const uint8_t* source, std::size_t height, std::size_t width, std::size_t stride,
        uint8_t* destination, std::size_t destination_size) {
    return prepare_hd_nv12(source, height, width, stride, destination, destination_size, 1);
}

#if defined(_OPENMP)
extern "C" int rescue_prepare_hd_nv12_parallel(
        const uint8_t* source, std::size_t height, std::size_t width, std::size_t stride,
        uint8_t* destination, std::size_t destination_size, int workers) {
    return prepare_hd_nv12(source, height, width, stride, destination, destination_size, workers);
}
#endif
