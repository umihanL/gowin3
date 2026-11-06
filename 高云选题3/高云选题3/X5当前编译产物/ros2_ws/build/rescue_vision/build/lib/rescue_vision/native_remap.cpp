// Exact fixed-point OpenCV INTER_LINEAR BGR remap, constant zero border.
#include <cstdint>
#include <cstddef>
#include <limits>
#if defined(_WIN32)
#define API extern "C" __declspec(dllexport)
#else
#define API extern "C" __attribute__((visibility("default")))
#endif
API int rescue_remap_version() { return 1; }
static bool overlap(const void* a, size_t na, const void* b, size_t nb) {
    const uintptr_t pa = reinterpret_cast<uintptr_t>(a), pb = reinterpret_cast<uintptr_t>(b);
    if (na > std::numeric_limits<uintptr_t>::max()-pa || nb > std::numeric_limits<uintptr_t>::max()-pb) return true;
    return pa < pb+nb && pb < pa+na;
}
API int rescue_remap_bgr(const uint8_t* src, size_t src_bytes, int width, int height,
    const int16_t* xy, size_t xy_bytes, const uint16_t* fractions, size_t fraction_bytes,
    int out_width, int out_height, uint8_t* dst, size_t dst_bytes, int threads) {
    if (!src || !xy || !fractions || !dst || width < 1 || height < 1 ||
        out_width < 1 || out_height < 1 || width >= 32767 || height >= 32767 ||
        out_width >= 32767 || out_height >= 32767 || threads < 1 || threads > 8) return -1;
    const size_t source_stride = static_cast<size_t>(width)*3;
    const size_t input_bytes = source_stride*height;
    const size_t pixels = static_cast<size_t>(out_width)*out_height;
    if (src_bytes < input_bytes || xy_bytes < pixels*4 || fraction_bytes < pixels*2 || dst_bytes < pixels*3) return -2;
    if (overlap(dst, pixels*3, src, input_bytes) || overlap(dst, pixels*3, xy, pixels*4) ||
        overlap(dst, pixels*3, fractions, pixels*2)) return -3;
    #pragma omp parallel for schedule(static) num_threads(threads)
    for (int row = 0; row < out_height; ++row) {
        size_t index = static_cast<size_t>(row)*out_width;
        uint8_t* output = dst + index*3;
        for (int column = 0; column < out_width; ++column, ++index, output += 3) {
            const int x = xy[2*index], y = xy[2*index+1];
            const int fraction = fractions[index] & 1023;
            const int fx = fraction & 31, fy = fraction >> 5;
            const int w0 = (32-fx)*(32-fy), w1 = fx*(32-fy), w2 = (32-fx)*fy, w3 = fx*fy;
            if (static_cast<unsigned>(x) < static_cast<unsigned>(width-1) &&
                static_cast<unsigned>(y) < static_cast<unsigned>(height-1)) {
                const uint8_t* upper = src + static_cast<size_t>(y)*source_stride + x*3;
                const uint8_t* lower = upper + source_stride;
                output[0] = (upper[0]*w0 + upper[3]*w1 + lower[0]*w2 + lower[3]*w3 + 512) >> 10;
                output[1] = (upper[1]*w0 + upper[4]*w1 + lower[1]*w2 + lower[4]*w3 + 512) >> 10;
                output[2] = (upper[2]*w0 + upper[5]*w1 + lower[2]*w2 + lower[5]*w3 + 512) >> 10;
            } else {
                int totals[3] = {512, 512, 512};
                const int weights[] = {w0, w1, w2, w3};
                for (int dy = 0; dy < 2; ++dy) {
                    if (y+dy < 0 || y+dy >= height) continue;
                    for (int dx = 0; dx < 2; ++dx) {
                        if (x+dx < 0 || x+dx >= width) continue;
                        const uint8_t* pixel = src + static_cast<size_t>(y+dy)*source_stride + (x+dx)*3;
                        const int weight = weights[2*dy+dx];
                        for (int c = 0; c < 3; ++c) totals[c] += pixel[c]*weight;
                    }
                }
                for (int c = 0; c < 3; ++c) output[c] = totals[c] >> 10;
            }
        }
    }
    return 0;
}
