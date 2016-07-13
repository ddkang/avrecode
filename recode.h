#pragma once

extern "C" {
#include "libavcodec/avcodec.h"
#include "libavcodec/cabac.h"
#include "libavcodec/coding_hooks.h"
#include "libavformat/avformat.h"
#include "libavformat/avio.h"
#include "libavutil/error.h"
#include "libavutil/file.h"
}

#include "arithmetic_code.h"
#include "cabac_code.h"
#include "recode.pb.h"
#include "framebuffer.h"



// CABAC blocks smaller than this will be skipped.
const int SURROGATE_MARKER_BYTES = 8;
// #define DO_NEIGHBOR_LOGGING
#ifdef DO_NEIGHBOR_LOGGING
#define LOG_NEIGHBORS printf
#else
#define LOG_NEIGHBORS(...)
#endif
template <typename T>
std::unique_ptr<T, std::function<void(T*&)>> av_unique_ptr(T* p, const std::function<void(T*&)>& deleter) {
  if (p == nullptr) {
    throw std::bad_alloc();
  }
  return std::unique_ptr<T, std::function<void(T*&)>>(p, deleter);
}
template <typename T>
std::unique_ptr<T, std::function<void(T*&)>> av_unique_ptr(T* p, void (*deleter)(T**)) {
  return av_unique_ptr<T>(p, [deleter](T*& to_delete){ deleter(&to_delete); });
}
template <typename T>
std::unique_ptr<T, std::function<void(T*&)>> av_unique_ptr(T* p, void (*deleter)(void*) = av_free) {
  return av_unique_ptr<T>(p, [deleter](T*& to_delete){ deleter(to_delete); });
}

template <typename T = std::function<void()>>
struct defer {
  T to_defer;
  explicit defer(const T& to_defer) : to_defer(to_defer) {}
  defer(const defer&) = delete;
  ~defer() { to_defer(); }
};

struct r_scan8 {
  uint16_t scan8_index;
  bool neighbor_left;
  bool neighbor_up;
  bool is_invalid() const {
      return scan8_index == 0 && neighbor_left && neighbor_up;
  }
  static constexpr r_scan8 inv() {
      return {0, true, true};
  }
};
/* Scan8 organization:
 *    0 1 2 3 4 5 6 7
 * 0  DY    y y y y y
 * 1        y Y Y Y Y
 * 2        y Y Y Y Y
 * 3        y Y Y Y Y
 * 4  du    y Y Y Y Y
 * 5  DU    u u u u u
 * 6        u U U U U
 * 7        u U U U U
 * 8        u U U U U
 * 9  dv    u U U U U
 * 10 DV    v v v v v
 * 11       v V V V V
 * 12       v V V V V
 * 13       v V V V V
 * 14       v V V V V
 * DY/DU/DV are for luma/chroma DC.
 */
constexpr uint8_t scan_8[16 * 3 + 3] = {
    4 +  1 * 8, 5 +  1 * 8, 4 +  2 * 8, 5 +  2 * 8,
    6 +  1 * 8, 7 +  1 * 8, 6 +  2 * 8, 7 +  2 * 8,
    4 +  3 * 8, 5 +  3 * 8, 4 +  4 * 8, 5 +  4 * 8,
    6 +  3 * 8, 7 +  3 * 8, 6 +  4 * 8, 7 +  4 * 8,
    4 +  6 * 8, 5 +  6 * 8, 4 +  7 * 8, 5 +  7 * 8,
    6 +  6 * 8, 7 +  6 * 8, 6 +  7 * 8, 7 +  7 * 8,
    4 +  8 * 8, 5 +  8 * 8, 4 +  9 * 8, 5 +  9 * 8,
    6 +  8 * 8, 7 +  8 * 8, 6 +  9 * 8, 7 +  9 * 8,
    4 + 11 * 8, 5 + 11 * 8, 4 + 12 * 8, 5 + 12 * 8,
    6 + 11 * 8, 7 + 11 * 8, 6 + 12 * 8, 7 + 12 * 8,
    4 + 13 * 8, 5 + 13 * 8, 4 + 14 * 8, 5 + 14 * 8,
    6 + 13 * 8, 7 + 13 * 8, 6 + 14 * 8, 7 + 14 * 8,
    0 +  0 * 8, 0 +  5 * 8, 0 + 10 * 8
};

constexpr r_scan8 reverse_scan_8[15][8] = {
    //Y
    {{16 * 3, false, false}, r_scan8::inv(), r_scan8::inv(), {15, true, true},
     {10, false, true}, {11, false, true}, {14, false, true}, {15, false, true}},
    {r_scan8::inv(), r_scan8::inv(), r_scan8::inv(), {5, true, false},
     {0, false, false}, {1, false, false}, {4, false, false}, {5, false, false}},
    {r_scan8::inv(), r_scan8::inv(), r_scan8::inv(), {7, true, false},
     {2, false, false}, {3, false, false}, {6, false, false}, {7, false, false}},
    {r_scan8::inv(), r_scan8::inv(), r_scan8::inv(), {13, true, false},
     {8, false, false}, {9, false, false}, {12, false, false}, {13, false, false}},
    {{16 * 3 + 1,false, true}, r_scan8::inv(), r_scan8::inv(), {15, true, false},
     {10, false, false}, {11, false, false}, {14, false, false}, {15, false, false}},
    // U
    {{16 * 3 + 1,false, false}, r_scan8::inv(), r_scan8::inv(), {16 + 15, true, true},
     {16 + 10, false, true}, {16 + 11, false, true}, {16 + 14, false, true}, {16 + 15, false, true}},
    {r_scan8::inv(), r_scan8::inv(), r_scan8::inv(), {16 + 5, true, false},
     {16 + 0, false, false}, {16 + 1, false, false}, {16 + 4, false, false}, {16 + 5, false, false}},
    {r_scan8::inv(), r_scan8::inv(), r_scan8::inv(), {16 + 7, true, false},
     {16 + 2, false, false}, {16 + 3, false, false}, {16 + 6, false, false}, {16 + 7, false, false}},
    {r_scan8::inv(), r_scan8::inv(), r_scan8::inv(), {16 + 13, true, false},
     {16 + 8, false, false}, {16 + 9, false, false}, {16 + 12, false, false}, {16 + 13, false, false}},
    {{16 * 3 + 2,false, true}, r_scan8::inv(), r_scan8::inv(), {16 + 15, true, false},
     {16 + 10, false, false}, {16 + 11, false, false}, {16 + 14, false, false}, {16 + 15, false, false}},
    // V
    {{16 * 3 + 2,false, false}, r_scan8::inv(), r_scan8::inv(), {32 + 15, true, true},
     {32 + 10, false, true}, {32 + 11, false, true}, {32 + 14, false, true}, {32 + 15, false, true}},
    {r_scan8::inv(), r_scan8::inv(), r_scan8::inv(), {32 + 5, true, false},
     {32 + 0, false, false}, {32 + 1, false, false}, {32 + 4, false, false}, {32 + 5, false, false}},
    {r_scan8::inv(), r_scan8::inv(), r_scan8::inv(), {32 + 7, true, false},
     {32 + 2, false, false}, {32 + 3, false, false}, {32 + 6, false, false}, {32 + 7, false, false}},
    {r_scan8::inv(), r_scan8::inv(), r_scan8::inv(), {32 + 13, true, false},
     {32 + 8, false, false}, {32 + 9, false, false}, {32 + 12, false, false}, {32 + 13, false, false}},
    {{32 + 16 * 3 + 1,false, true}, r_scan8::inv(), r_scan8::inv(), {32 + 15, true, false},
     {32 + 10, false, false}, {32 + 11, false, false}, {32 + 14, false, false}, {32 + 15, false, false}}};

// Encoder / decoder for recoded CABAC blocks.
typedef uint64_t range_t;
typedef arithmetic_code<range_t, uint8_t> recoded_code;

/*
not sure these tables are the ones we want to use
constexpr uint8_t unzigzag16[16] = {
    0 + 0 * 4, 0 + 1 * 4, 1 + 0 * 4, 0 + 2 * 4,
    0 + 3 * 4, 1 + 1 * 4, 1 + 2 * 4, 1 + 3 * 4,
    2 + 0 * 4, 2 + 1 * 4, 2 + 2 * 4, 2 + 3 * 4,
    3 + 0 * 4, 3 + 1 * 4, 3 + 2 * 4, 3 + 3 * 4,
};
constexpr uint8_t zigzag16[16] = {
    0, 2, 8, 12,
    1, 5, 9, 13,
    3, 6, 10, 14,
    4, 7, 11, 15
};

constexpr uint8_t zigzag_field64[64] = {
    0 + 0 * 8, 0 + 1 * 8, 0 + 2 * 8, 1 + 0 * 8,
    1 + 1 * 8, 0 + 3 * 8, 0 + 4 * 8, 1 + 2 * 8,
    2 + 0 * 8, 1 + 3 * 8, 0 + 5 * 8, 0 + 6 * 8,
    0 + 7 * 8, 1 + 4 * 8, 2 + 1 * 8, 3 + 0 * 8,
    2 + 2 * 8, 1 + 5 * 8, 1 + 6 * 8, 1 + 7 * 8,
    2 + 3 * 8, 3 + 1 * 8, 4 + 0 * 8, 3 + 2 * 8,
    2 + 4 * 8, 2 + 5 * 8, 2 + 6 * 8, 2 + 7 * 8,
    3 + 3 * 8, 4 + 1 * 8, 5 + 0 * 8, 4 + 2 * 8,
    3 + 4 * 8, 3 + 5 * 8, 3 + 6 * 8, 3 + 7 * 8,
    4 + 3 * 8, 5 + 1 * 8, 6 + 0 * 8, 5 + 2 * 8,
    4 + 4 * 8, 4 + 5 * 8, 4 + 6 * 8, 4 + 7 * 8,
    5 + 3 * 8, 6 + 1 * 8, 6 + 2 * 8, 5 + 4 * 8,
    5 + 5 * 8, 5 + 6 * 8, 5 + 7 * 8, 6 + 3 * 8,
    7 + 0 * 8, 7 + 1 * 8, 6 + 4 * 8, 6 + 5 * 8,
    6 + 6 * 8, 6 + 7 * 8, 7 + 2 * 8, 7 + 3 * 8,
    7 + 4 * 8, 7 + 5 * 8, 7 + 6 * 8, 7 + 7 * 8,
};

*/
constexpr uint8_t zigzag4[4] = {
    0, 1, 2, 3
};
constexpr uint8_t unzigzag4[4] = {
    0, 1, 2, 3
};

constexpr uint8_t unzigzag16[16] = {
    0,  1,  4,  8,
    5,  2,  3,  6,
    9, 12, 13, 10,
    7, 11, 14, 15
};
constexpr uint8_t ffmpeg_j_to_ind[16] = {
    0,  2,  3,  9,
    1,  4,  8, 10,
    5,  7, 11, 14,
    6, 12, 13, 15,
};
constexpr uint8_t ffmpeg_ind_to_j[16] = {
    0,   4,  1,  2,
    5,   8, 12,  9,
    6,   3,  7, 10,
    13, 14, 11, 15,
};
constexpr uint8_t zigzag16[16] = {
    0,  1,  5,  6,
    2,  4,  7, 12,
    3,  8, 11, 13,
    9, 10, 14, 15
};
constexpr uint8_t unzigzag64[64] = {
    0,   1,  8, 16,  9,  2,  3, 10,
    17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34,
    27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36,
    29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46,
    53, 60, 61, 54, 47, 55, 62, 63
};
constexpr uint8_t zigzag64[64] = {
    0,  1,  5,  6,  14, 15, 27, 28,
    2,  4,  7,  13, 16, 26, 29, 42,
    3,  8,  12, 17, 25, 30, 41, 43,
    9,  11, 18, 24, 31, 40, 44, 53,
    10, 19, 23, 32, 39, 45, 52, 54,
    20, 22, 33, 38, 46, 51, 55, 60,
    21, 34, 37, 47, 50, 56, 59, 61,
    35, 36, 48, 49, 57, 58, 62, 63
};
constexpr uint8_t ffmpeg8x8_ind_to_j[64] = {
    0,  8,  1,  2,  9, 16, 24, 17,
    10,  3,  4, 11, 18, 25, 32, 40,
    33, 26, 19, 12,  5,  6, 13, 20,
    27, 34, 41, 48, 56, 49, 42, 35,
    28, 21, 14,  7, 15, 22, 29, 36,
    43, 50, 57, 58, 51, 44, 37, 30,
    23, 31, 38, 45, 52, 59, 60, 53,
    46, 39, 47, 54, 61, 62, 55, 63,
};



struct CoefficientCoord {
  int mb_x;
  int mb_y;
  int scan8_index;
  int zigzag_index;
};

int log2(int y) {
  int x = -1;
  while (y) {
    y /= 2;
    x++;
  }
  return x;
}

int av_check(int return_value, int expected_error = 0, const std::string& message = "") {
  if (return_value >= 0 || return_value == expected_error) {
    return return_value;
  } else {
    char err[AV_ERROR_MAX_STRING_SIZE];
    av_make_error_string(err, AV_ERROR_MAX_STRING_SIZE, return_value);
    throw std::runtime_error(message + ": " + err);
  }
}
bool av_check(int return_value, const std::string& message = "") {
  return av_check(return_value, 0, message);
}

static int NUM_2X2 = 0, NUM_4X4 = 0, NUM_8X8 = 0;
static int NUM_WEIRD = 0, TOTAL_NUM = 0;
static bool flip = true;