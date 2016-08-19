extern "C" {
#include "libavcodec/avcodec.h"
#include "libavcodec/cabac.h"
#include "libavcodec/coding_hooks.h"
#include "libavformat/avformat.h"
#include "libavformat/avio.h"
#include "libavutil/error.h"
#include "libavutil/file.h"
}

#include <algorithm>

#include <string.h>
#include <stdlib.h>

#include "arithmetic_code.h"
#include "cabac_code.h"
#include "recode.pb.h"
#include "framebuffer.h"
#include "recode.h"
#include "nd_array.h"
#include "neighbors.h"
#include "helpers.h"

#pragma once

struct estimator {
  int pos = 1, neg = 1;
  void renormalize() {
    pos = (pos + 1) / 2;
    neg = (neg + 1) / 2;
  }
};

class EstimatorContext {
 public:
  /* We DO NOT want to clear the estimator every time begin is called.
   * This is to keep memory from call to call.
   * Also constructing and deconstructing these are slow.
   * Thus, begin SHOULD NOT make new estimators.
   */

  // FIXME: The parameter names are carried over from a poor choice.
  virtual void begin(const int zz_index, const int param0, const int param1) = 0;

  virtual CodingType update(const int symbol, const int context) = 0;

  virtual estimator* get_estimator(const int context) = 0;

  // Annoyingly, if everything is static, in persists in the roundtrip
  // So, clear state
  static void reset() {
    mb_coord = {};
    frames[0].bzero();
    frames[1].bzero();
    cur_frame = 0;
    nonzeros_observed = 0;
  }

  static void set_sub_mb_cat(const int sub_mb_cat_) {
    assert(sub_mb_cat_ < 6);
    sub_mb_cat = sub_mb_cat_;
  }

  static void set_sub_mb_size(const int sub_mb_size_) {
    sub_mb_size = sub_mb_size_;
  }

  static void set_sub_mb_is_dc(const int is_dc) {
    sub_mb_is_dc = is_dc;
  }

  static void set_mb_type(const int ff_mb_type) {
    BlockMeta &meta = frames[cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y);
    mb_type = parse_ff_mb_type(ff_mb_type);
    meta.mb_type = mb_type;
  }

  static void set_scan8_index(const int scan8_index) {
    mb_coord.scan8_index = scan8_index;
  }

  static void set_mb_xy(const int x, const int y) {
    mb_coord.mb_x = x;
    mb_coord.mb_y = y;
  }

  static int eob_symbol() {
    int num_nonzeros = frames[cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y).num_nonzeros[mb_coord.scan8_index];
    return num_nonzeros == nonzeros_observed;
  }

  static void update_frame_spec(const int frame_num, const int mb_width, const int mb_height) {
    if (frames[cur_frame].width() != (uint32_t) mb_width
        || frames[cur_frame].height() != (uint32_t) mb_height
        || !frames[cur_frame].is_same_frame(frame_num)) {
      cur_frame = !cur_frame;
      if (frames[cur_frame].width() != (uint32_t) mb_width
          || frames[cur_frame].height() != (uint32_t) mb_height) {
        frames[cur_frame].init(mb_width, mb_height, mb_width * mb_height);
        if (frames[!cur_frame].width() != (uint32_t) mb_width
            || frames[!cur_frame].height() != (uint32_t) mb_height) {
          frames[!cur_frame].init(mb_width, mb_height, mb_width * mb_height);
        }
        //fprintf(stderr, "Init(%d=%d) %d x %d\n", frame_num, cur_frame, mb_width, mb_height);
      } else {
        frames[cur_frame].bzero();
        //fprintf(stderr, "Clear (%d=%d)\n", frame_num, cur_frame);
      }
      frames[cur_frame].set_frame_num(frame_num);
    }
  }

  static void copy_coefficients(int16_t *block, int max_coeff) {
    int copy_size = max_coeff + (max_coeff == 15);
    if (max_coeff == 4) copy_size = 64;  // TODO: ffmpeg does something weird with chroma DCs
    memcpy(&frames[cur_frame].at(mb_coord.mb_x, mb_coord.mb_y).
               coeffs[mb_coord.scan8_index * 16],
           block,
           copy_size * sizeof(int16_t));
  }

  static void print_coeffs() {
    /*int16_t *block = &frames[cur_frame].at(mb_coord.mb_x, mb_coord.mb_y).
        coeffs[mb_coord.scan8_index * 16];

    if (frames[cur_frame].get_frame_num() > 1) return;
    // I suspect ffmpeg reserves space for 444 and only uses these for 420
    static const uint8_t chroma_scan[4] = {0, 16, 32, 48};
    const uint8_t *scan = (sub_mb_size > 4) ? ffmpeg_ind_to_j : chroma_scan;
    scan += (sub_mb_size == 15);

    if (sub_mb_size == 64) scan = ffmpeg8x8_ind_to_j;

    for (int ind = sub_mb_size-1 + (sub_mb_size == 15); ind >= 0; ind--) {
      int j = scan[ind];
      if (block[j]) printf("%d,%d,%d,%d,%d,%d,%d,%d\n",
                           frames[cur_frame].get_frame_num(),
                           mb_coord.mb_x, mb_coord.mb_y, mb_coord.scan8_index,
                           ind, j, block[j], sub_mb_size);
    }*/
  }

  // For now
  static constexpr int bypass_context = -1, terminate_context = -2;
  static constexpr int significance_context = -3, eob_context = -4;
  static constexpr int sign_bypass_context = -5;

 protected:
  static int mb_type, sub_mb_size, sub_mb_cat;
  static bool sub_mb_is_dc;
  static CoefficientCoord mb_coord;
  static FrameBuffer frames[2];
  static int cur_frame;

  // FIXME: this is actually a really horrible way to do this.
  // eob and significance both need this, which ruins my life
  static int nonzeros_observed;
};

class CABACGenericEst : public EstimatorContext {
 public:
  void begin(const int zz_index, const int param0, const int param1) {}
  CodingType update(const int symbol, const int context) {
    return PIP_UNKNOWN;
  }
  estimator* get_estimator(const int context) {
    switch (context) {
      case bypass_context:
        return &bypass_estimator;
      case sign_bypass_context:
        return &sign_bypass_esimator;
      case terminate_context:
        return &terminate_estimator;
      case eob_context:
        assert(false);  // should already be caught elsewhere
      default:
        if (context >= 0 && context < CABAC_STATE_SIZE) {
          return &cabac_estimator[context];
        } else {
          fprintf(stderr, "UHOH %d\n", context);
          assert(false);
        }
    }
  }

 private:
  const int CABAC_STATE_SIZE = 1024; // FIXME
  estimator bypass_estimator;
  estimator sign_bypass_esimator;
  estimator terminate_estimator;
  estimator cabac_estimator[1024]; // FIXME?
};


// Both code paths go through this prior
// However, index hurts for CABAC, so it's set to 0 on the ffmpeg side.
class Intra4x4PredModeEst : public EstimatorContext {
 public:
  void begin(const int zz_index, const int param0, const int param1) {
    bit_num = 0;
    running = 0;
    mode = param0;
    index = param1;
  }

  CodingType update(const int symbol, const int context) {
    if (bit_num) {
      running <<= 1;
      running |= symbol;
    }
    bit_num++;
    return PIP_INTRA4X4_PRED_MODE;
  }

  estimator* get_estimator(const int context) {
    if (!bit_num)
      return &skip_est[mode];
    else
      return &est[index][mode][running][bit_num - 1];
  }

 private:
  int bit_num, running, mode, index;
  estimator skip_est[9];
  estimator est[16][9][9][3];
};



class CABACChromaPreModeEst : public EstimatorContext {
 public:
  void begin(const int zz_index, const int param0, const int param1) {
    bit_num = 0;
  }
  CodingType update(const int symbol, const int context) {
    bit_num++;
    return PIP_MB_CHROMA_PRE_MODE;
  }
  estimator* get_estimator(const int context) {
    return &est[bit_num];
  }

 private:
  estimator est[3];
  int bit_num = -1;
};

class CABACCodedBlockEst : public EstimatorContext {
 public:
  void begin(const int zz_index, const int param0, const int param1) {
    nza = !!param0;
    nzb = !!param1;
  }
  CodingType update(const int symbol, const int context) {
    return PIP_CODED_BLOCK;
  }
  estimator* get_estimator(const int context) {
    return &est[sub_mb_cat][mb_coord.scan8_index][nza][nzb];
  }

 private:
  estimator est[6][64][2][2];
  int nza, nzb;
};

class CABACIntraMBTypeEst : public EstimatorContext {
 public:
  void begin(const int intra_slice_, const int param0, const int param1) {
    mb_left = parse_ff_mb_type(param0);
    mb_top = parse_ff_mb_type(param1);
    intra_slice = intra_slice_;
    bit_num = 0;
    mb_cbp_chroma = false;
  }
  CodingType update(const int symbol, const int context) {
    if (bit_num == 2)
      mb_cbp_chroma = symbol;
    if (context != terminate_context)
      bit_num++;
    return PIP_INTRA_MB_TYPE;
  }
  estimator* get_estimator(const int context) {
    if (context == terminate_context) {
      nearly_zero.pos = 1;
      nearly_zero.neg = 2047;
      return &nearly_zero;
    }
    return &est[mb_left][mb_top][intra_slice][bit_num][mb_cbp_chroma];
  }
 private:
  int mb_left, mb_top, bit_num;
  bool intra_slice, mb_cbp_chroma;
  estimator est[14][14][2][5][2];
  estimator nearly_zero;
};

class CABACCbpChromaEst : public EstimatorContext {
 public:
  void begin(const int zz_index, const int param0, const int param1) {
    bit_num = 0;
    left = (param0 >> 4) & 0x03;
    top = (param1 >> 4) & 0x03;
  }
  CodingType update(const int symbol, const int context) {
    bit_num++;
    return PIP_MB_CBP_CHROMA;
  }
  estimator* get_estimator(const int context) {
    return &est[bit_num][left][top];
  }
 private:
  int bit_num, left, top;
  estimator est[2][4][4];
};

class CABACMBSkipEst : public EstimatorContext {
 public:
  void begin(const int zz_index, const int param0, const int param1) {
    left = mb_coord.mb_x != 0;
    if (left) left += frames[cur_frame].meta_at(mb_coord.mb_x - 1, mb_coord.mb_y).coded;
    top = mb_coord.mb_y != 0;
    if (top) top += frames[cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y - 1).coded;
    last = frames[cur_frame].get_frame_num() != 0;
    if (last) last += frames[!cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y).coded;
  }
  CodingType update(const int symbol, const int context) {
    return PIP_MB_SKIP_FLAG;
  }
  estimator* get_estimator(const int context) {
    return &est[left][top][last];
  }

 private:
  int left, top, last;
  estimator est[3][3][3];
};

class CABACCbpLumaEst : public EstimatorContext {
 public:
  void begin(const int zz_index, const int param0, const int param1) {
    bit_num = last = running = 0;
  }
  CodingType update(const int symbol, const int context) {
    last = symbol;
    running += symbol << bit_num;
    bit_num++;
    return PIP_MB_CBP_LUMA;
  }
  estimator* get_estimator(const int context) {
    return &est[0][mb_type][0];
    /*int left = mb_coord.mb_x != 0;
    if (left) left += frames[cur_frame].meta_at(mb_coord.mb_x - 1, mb_coord.mb_y).cbp_luma;
    int top = mb_coord.mb_y != 0;
    if (top) top += frames[cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y - 1).cbp_luma;
    int last = frames[cur_frame].get_frame_num() != 0;
    if (last) last += frames[!cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y).cbp_luma;*/
    // losslessh264 uses the block type, which we do not currently track.
    // int mb_type = frames[cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y).mb_type;
    // int prev_mb_type = frames[!cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y).mb_type;

    // return cbp_luma.get_estimator(context);

    /*if (mb_type == prev_mb_type)
      return &mb_cbp_luma[last][mb_type][cbp_luma_bit_num];
    else
      return generic_est.get_estimator(context);*/
  }

 private:
  int bit_num, last, running;
  estimator est[17][17][4];
};

class CABACResidualsEst : public EstimatorContext {
 public:
  CABACResidualsEst() {
    est = std::make_unique<est_array>();
  }
  void begin(const int zz_index, const int param0, const int param1) {
    index = param0;
    bit_num = 0; // ??
  }
  CodingType update(const int symbol, const int context) {
    bit_num++;
    if (context == sign_bypass_context)
      bit_num = 0;
    return PIP_RESIDUALS;
  }
  estimator* get_estimator(const int context) {
    if (context == sign_bypass_context) {
      return &sign_est[sub_mb_cat][index];
    } else if (context == bypass_context) {
      return &bypass_est;
    } else {
      if (!bit_num)
        return &is_one_est[sub_mb_cat][index];
      else
        return &est->at(sub_mb_cat, index, std::min(bit_num - 1, 16));
    }
  }

 private:
  int index, bit_num;
  estimator sign_est[14][64];
  estimator bypass_est;
  // This should rarely occur
  estimator is_one_est[14][64];
  typedef Sirikata::Array3d<estimator, 6, 64, 17> est_array;
  std::unique_ptr<est_array> est;
};

class CABACMBMvdEst : public EstimatorContext {
 public:
  void begin(const int zz_index, const int param0, const int param1) {
    ind = param0;
    bit_num = 0;
    state = 0;
  }
  CodingType update(const int symbol, const int context) {
    if (context == bypass_context && state == 0) {
      state++;
      bit_num = -1;
    }
    if (context == bypass_context && state == 1 && symbol == 0) {
      state++;
      bit_num = -1;
    }
    bit_num++;
    return PIP_MB_MVD;
  }
  estimator* get_estimator(const int context) {
    if (context == sign_bypass_context) {
      return &sign_est[0][0]; // what lol
    } else if (context == bypass_context) {
      int clipped_bit_num = std::min(bit_num, 9);
      return &exp_est[state - 1][clipped_bit_num];
    } else {
      int clipped_num = std::min(bit_num, 10);
      return &est[ind][clipped_num][mb_type];
    }
  }
 private:
  bool ind;
  int bit_num;
  int state; // unary, exp, rev exp -- change to enum?
  estimator est[2][9][12];
  estimator sign_est[2][12];
  estimator exp_est[2][10];
};


class CABACSignificanceEOBEst : public EstimatorContext {
 public:
  void begin(const int zz_index, const int param0, const int param1) {}

  CodingType update(const int symbol, const int context) {
    CodingType coding_type = PIP_SIGNIFICANCE_EOB;
    if (symbol) {
      mb_coord.zigzag_index = 0;
      coding_type = PIP_UNREACHABLE;
    } else if (mb_coord.zigzag_index + 2 == sub_mb_size) {
      frames[cur_frame].at(mb_coord.mb_x, mb_coord.mb_y).residual[mb_coord.scan8_index * 16 +
                                                                  mb_coord.zigzag_index + 1] = 1;
      coding_type = PIP_UNREACHABLE;
    } else {
      coding_type = PIP_SIGNIFICANCE_MAP;
      ++mb_coord.zigzag_index;
    }

    return coding_type;
  }

  estimator *get_estimator(const int context) {
    return &eob_estimator[eob_symbol()];
  }

 private:
  estimator eob_estimator[2];
};

class CABACSignificanceMapEst : public EstimatorContext {
 public:
  CABACSignificanceMapEst() {
    significance_estimator = std::make_unique<sig_array>();
    queue_estimators = std::make_unique<queue_array>();
  }

  void begin(const int zz_index, const int param0, const int param1) {
    BlockMeta &meta = frames[cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y);
    meta.num_nonzeros[mb_coord.scan8_index] = 0;

    assert(!zz_index);
    nonzeros_observed = 0;
    mb_coord.zigzag_index = 0;
  }

  void reset_mb_significance_state_tracking() {
    mb_coord.zigzag_index = 0;
    nonzeros_observed = 0;
  }

  CodingType update(const int symbol, const int context) {
    CodingType coding_type = PIP_SIGNIFICANCE_MAP;
    frames[cur_frame].at(mb_coord.mb_x, mb_coord.mb_y).
        residual[mb_coord.scan8_index * 16 + mb_coord.zigzag_index] = symbol;

    nonzeros_observed += symbol;
    if (mb_coord.zigzag_index + 1 == sub_mb_size) {
      coding_type = PIP_UNREACHABLE;
      mb_coord.zigzag_index = 0;
    } else {
      if (symbol) {
        coding_type = PIP_SIGNIFICANCE_EOB;
      } else {
        ++mb_coord.zigzag_index;
        if (mb_coord.zigzag_index + 1 == sub_mb_size) {
          // If zero and haven't EOB'd, then the next and last must be 1
          frames[cur_frame].at(mb_coord.mb_x, mb_coord.mb_y).residual[mb_coord.scan8_index * 16 +
                                                                      mb_coord.zigzag_index] = 1;
          ++nonzeros_observed;
          coding_type = PIP_UNREACHABLE;
          mb_coord.zigzag_index = 0;
        }
      }
    }

    return coding_type;
  }

  template<class Functor>
  void finish_queueing(const Functor &put_or_get) {
    BlockMeta &meta = frames[cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y);
    int nonzero_bits[7] = {};
    for (int i = 0; i < 7; ++i)
      nonzero_bits[i] = (meta.num_nonzeros[mb_coord.scan8_index] & (1 << i)) >> i;

#define QUEUE_MODE
#ifdef QUEUE_MODE
    const uint32_t serialized_bits = sub_mb_size > 16 ? 7 : sub_mb_size > 4 ? 5 : 3;
    {
      uint32_t i = 0;
      uint32_t serialized_so_far = 0;
      CoefficientCoord neighbor;
      uint32_t left_nonzero = 0;
      uint32_t above_nonzero = 0;
      bool has_above = get_neighbor_sub_mb(true, sub_mb_size, mb_coord, &neighbor);
      if (has_above) {
        above_nonzero = frames[cur_frame].meta_at(neighbor.mb_x, neighbor.mb_y).num_nonzeros[neighbor.scan8_index];
      }
      bool has_left = get_neighbor_sub_mb(false, sub_mb_size, mb_coord, &neighbor);
      if (has_left) {
        left_nonzero = frames[cur_frame].meta_at(neighbor.mb_x, neighbor.mb_y).num_nonzeros[neighbor.scan8_index];
      }

      do {
        uint32_t cur_bit = (1 << i);
        uint32_t tmp = serialized_so_far | cur_bit;
        int left_nonzero_bit = 2;
        if (has_left) left_nonzero_bit = (left_nonzero >= tmp);
        int above_nonzero_bit = 2;
        if (has_above) above_nonzero_bit = (above_nonzero >= tmp);
        auto *e = &queue_estimators->at(i, serialized_so_far,
                                        (frames[!cur_frame].meta_at(
                                            mb_coord.mb_x, mb_coord.mb_y).num_nonzeros[mb_coord.scan8_index] >= tmp),
                                        left_nonzero_bit,
                                        above_nonzero_bit,
                                        sub_mb_cat);
        put_or_get(e, &nonzero_bits[i], i);

        if (nonzero_bits[i])
          serialized_so_far |= cur_bit;
      } while (++i < serialized_bits);
    }
#endif
    meta.num_nonzeros[mb_coord.scan8_index] = 0;
    for (int i = 0; i < 7; ++i)
      meta.num_nonzeros[mb_coord.scan8_index] |= nonzero_bits[i] << i;
  }

  void end_coding_type(CodingType coding_type) {
    assert(coding_type == PIP_UNREACHABLE
           || (coding_type == PIP_SIGNIFICANCE_MAP && mb_coord.zigzag_index == 0));
    uint8_t num_nonzeros = 0;
    for (int i = 0; i < sub_mb_size + (sub_mb_size == 15); ++i) {
      int16_t res = frames[cur_frame].at(mb_coord.mb_x, mb_coord.mb_y).residual[mb_coord.scan8_index * 16 + i];
      assert(res == 1 || res == 0);
      if (res != 0)
        num_nonzeros += 1;
    }
    BlockMeta &meta = frames[cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y);
    meta.is_8x8 = (sub_mb_size > 32); // 8x8 will have DC be 2x2
    meta.coded = true;
    assert(meta.num_nonzeros[mb_coord.scan8_index] == 0 || meta.num_nonzeros[mb_coord.scan8_index] == num_nonzeros);
    meta.num_nonzeros[mb_coord.scan8_index] = num_nonzeros;
    meta.sub_mb_size = sub_mb_size;
  }

  estimator* get_estimator(const int context) {
    static const uint8_t sig_coeff_flag_offset_8x8[2][63] = {
        {0, 1, 2, 3, 4, 5, 5, 4, 4, 3, 3, 4, 4, 4, 5, 5,
            4, 4, 4,  4,  3, 3,  6,  7,  7, 7, 8,  9,  10, 9,  8,  7,
            7, 6, 11, 12, 13, 11, 6,  7,  8, 9, 14, 10, 9, 8,  6,  11,
            12, 13, 11, 6, 9,  14, 10, 9, 11, 12, 13, 11, 14, 10, 12},
        {0, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 7, 7, 8, 4, 5,
            6, 9, 10, 10, 8, 11, 12, 11, 9, 9, 10, 10, 8,  11, 12, 11,
            9, 9, 10, 10, 8,  11, 12, 11, 9, 9, 10, 10, 8, 13, 13, 9,
            9,  10, 10, 8, 13, 13, 9,  9, 10, 10, 14, 14, 14, 14, 14}
    };
    static const uint8_t sig_coeff_offset_dc[7] = {0, 0, 1, 1, 2, 2, 2};

    int zigzag_offset = mb_coord.zigzag_index;
    // For chroma422 which we do not support
    if (sub_mb_is_dc && false) {
      assert(mb_coord.zigzag_index < 7);
      zigzag_offset = sig_coeff_offset_dc[mb_coord.zigzag_index];
    } else {
      if (sub_mb_size > 32) {
        assert(mb_coord.zigzag_index < 63);
        zigzag_offset = sig_coeff_flag_offset_8x8[0][mb_coord.zigzag_index];
      }
    }
    int num_nonzeros = frames[cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y).num_nonzeros[mb_coord.scan8_index];
    return &significance_estimator->at(num_nonzeros - nonzeros_observed,
                                       zigzag_offset,
                                       sub_mb_cat,
                                       mb_type, 0, 0, 0);
  }

 private:
  typedef Sirikata::Array7d<estimator, 64, 64, 6, 12, 1, 1, 1> sig_array;
  typedef Sirikata::Array6d<estimator, 7, 64, 2, 3, 3, 6> queue_array;
  std::unique_ptr<sig_array> significance_estimator;
  std::unique_ptr<queue_array> queue_estimators;
};

int EstimatorContext::sub_mb_cat = -1;
int EstimatorContext::sub_mb_size = -1;
bool EstimatorContext::sub_mb_is_dc = false;
int EstimatorContext::mb_type = -1;
CoefficientCoord EstimatorContext::mb_coord;
FrameBuffer EstimatorContext::frames[2] = {};
int EstimatorContext::cur_frame = 0;
int EstimatorContext::nonzeros_observed = 0;
constexpr int EstimatorContext::bypass_context, EstimatorContext::terminate_context;
constexpr int EstimatorContext::significance_context, EstimatorContext::eob_context;
constexpr int EstimatorContext::sign_bypass_context;