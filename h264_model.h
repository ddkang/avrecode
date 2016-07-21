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


#define STRINGIFY_COMMA(s) #s ,
const char * billing_names [] = {EACH_PIP_CODING_TYPE(STRINGIFY_COMMA)};
#undef STRINGIFY_COMMA

class h264_model {
 public:
  struct estimator {
    int pos = 1, neg = 1;
    void renormalize() {
      pos = (pos + 1) / 2;
      neg = (neg + 1) / 2;
    }
  };

 private:
  typedef Sirikata::Array7d<estimator, 64, 64, 14, 12, 6, 1, 1> sig_array;
  typedef Sirikata::Array6d<estimator, 6, 32, 2, 3, 3, 6> queue_array;
  const int CABAC_STATE_SIZE = 1024;  // FIXME

  std::unique_ptr<sig_array> significance_estimator;
  std::unique_ptr<queue_array> queue_estimators;

  estimator bypass_estimator;
  estimator sign_bypass_esimator;
  estimator terminate_estimator;
  estimator eob_estimator[2];

  // ltype, ttype, intra_slice, bit_num, cbp_chroma
  estimator intra_mb_type_est[14][14][2][5][2];

  estimator intra4x4_pred_mode_skip[16];
  estimator intra4x4_pred_mode_estimator[3][16][16];

  estimator mb_skip_estimator[3][3][3];
  estimator mb_cbp_luma[17][17][4]; // Maybe one more dimension?

  estimator mb_mvd_estimator[2][9][12];
  estimator mb_mvd_sign_est[2][12];
  estimator mb_mvd_exp_est[2][10]; // FIXME: can this actually be 21 bits long?

  estimator residuals_sign_est[14][64];
  estimator residuals_bypass_est; // This should almost never occur
  estimator residuals_is_one_est[14][64];
  typedef Sirikata::Array3d<estimator, 6, 64, 17> residuals_est_array;
  std::unique_ptr<residuals_est_array> residuals_est;

  estimator coded_block_est[6][64][2][2];

  estimator cabac_estimator[1024]; // FIXME

  estimator nearly_one;
  estimator nearly_zero;

 public:
  static constexpr int bypass_context = -1, terminate_context = -2;
  static constexpr int significance_context = -3, eob_context = -4;
  static constexpr int sign_bypass_context = -5;

  size_t bill[sizeof(billing_names) / sizeof(billing_names[0])];
  size_t cabac_bill[sizeof(billing_names) / sizeof(billing_names[0])];

  // Global context
  CodingType coding_type = PIP_UNKNOWN;
  FrameBuffer frames[2];
  int cur_frame = 0;
  bool do_print;
  CoefficientCoord mb_coord;

  // intra MB type context
  int intra_mb_left = 0;
  int intra_mb_top = 0;
  bool intra_slice = false;
  int intra_mb_bit_num = 0;
  bool intra_mb_cbp_chroma = false;

  // Coded block context
  int cbf_nza = 0;
  int cbf_nzb = 0;

  // Residuals context
  int residuals_bit_num = 0;
  int residual_index = 0;

  // mvd context
  bool mvd_ind = 0; // This can only take two values.
  int mvd_bit_num = 0;
  int mvd_state; // unary, exp, rev exp

  // cbp luma context
  int cbp_luma_bit_num = 0;
  int cbp_luma_last = 0;
  int cbp_luma_running = 0;

  // intra4x4_pred_mode context
  int intra4x4_pred_mode_bit_num = 0;
  int intra4x4_pred_mode_last = 0;
  int intra4x4_pred_mode_running = 0;
  int intra4x4_pred_mode_last_pred = 0;

  // Significance map context
  int nonzeros_observed = 0;
  int sub_mb_cat = -1;
  int sub_mb_size = -1;
  int sub_mb_is_dc = 0;
  int sub_mb_chroma422 = 0;

 public:
  h264_model() {
    reset();
    do_print = false;
    memset(bill, 0, sizeof(bill));
    memset(cabac_bill, 0, sizeof(cabac_bill));

    significance_estimator = std::make_unique<sig_array>();
    queue_estimators = std::make_unique<queue_array>();
    residuals_est = std::make_unique<residuals_est_array>();

    deserialize_est(significance_estimator->begin(), std::string("../models/sig_est.bin"));
    deserialize_est(queue_estimators->begin(), std::string("../models/queue_est.bin"));
  }

  void enable_debug() {
    do_print = true;
  }

  void disable_debug() {
    do_print = false;
  }

  void serialize_est(estimator *begin, estimator *end, std::string fname) {
    const size_t len = end - begin;
    std::string contents((char*) begin, len * sizeof(estimator));

    // Force flush and deconstruct.
    {
      std::ofstream out_file;
      out_file.open(fname);
      out_file << contents;
      out_file.flush();
    }
  }

  // This assumes the size of the estimator matches the size of the file.
  void deserialize_est(estimator *begin, std::string fname) {
    std::ifstream fin(fname);
    std::string contents( (std::istreambuf_iterator<char>(fin)),
                           (std::istreambuf_iterator<char>()));

    memcpy((void *) begin, contents.c_str(), contents.length());
  }

  ~h264_model() {
    bool first = true;
    size_t total = 0;
    for (size_t i = 0; i < sizeof(billing_names) / sizeof(billing_names[i]); ++i) {
      if (bill[i]) {
        if (first) {
          fprintf(stderr, "Avrecode Bill\n=============\n");
        }
        first = false;
        fprintf(stderr, "%s : %ld\n", billing_names[i], bill[i]);
        total += bill[i];
      }
    }
    fprintf(stderr, "TOTAL : %ld\n", total);
    for (size_t i = 0; i < sizeof(billing_names) / sizeof(billing_names[i]); ++i) {
      if (cabac_bill[i]) {
        if (first) {
          fprintf(stderr, "CABAC Bill\n=============\n");
        }
        first = false;
        fprintf(stderr, "%s : %ld\n", billing_names[i], cabac_bill[i]);
      }
    }

    // serialize_est(significance_estimator->begin(), significance_estimator->end(), std::string("../models/sig_est.bin"));
    // serialize_est(queue_estimators->begin(), queue_estimators->end(), std::string("../models/queue_est.bin"));
  }

  void billable_bytes(size_t num_bytes_emitted) {
    bill[coding_type] += num_bytes_emitted;
  }

  void billable_cabac_bytes(size_t num_bytes_emitted) {
    cabac_bill[coding_type] += num_bytes_emitted;
  }

  void reset() {
    // reset should do nothing as we wish to remember what we've learned
  }

  bool fetch(bool previous, bool match_type, CoefficientCoord coord, int16_t *output) const {
    if (match_type && (previous || coord.mb_x != mb_coord.mb_x || coord.mb_y != mb_coord.mb_y)) {
      BlockMeta meta = frames[previous ? !cur_frame : cur_frame].meta_at(coord.mb_x, coord.mb_y);
      if (!meta.coded) { // when we populate mb_type in the metadata, then we can use it here
        return false;
      }
    }
    *output = frames[previous ? !cur_frame : cur_frame].
        at(coord.mb_x, coord.mb_y).residual[coord.scan8_index * 16 +
                                            coord.zigzag_index + (sub_mb_size == 15)];
    return true;
  }


  int eob_symbol() const {
    int num_nonzeros = frames[cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y).num_nonzeros[mb_coord.scan8_index];
    return num_nonzeros == nonzeros_observed;
  }

  range_t probability_for_model_key(range_t range, estimator *e) {
    int total = e->pos + e->neg;
    return (range / total) * e->pos;
  }

  range_t probability_for_state(range_t range, int context, int symbol) {
    auto *e = get_estimator(context, symbol);
    return probability_for_model_key(range, e);
  }

  void update_frame_spec(int frame_num, int mb_width, int mb_height) {
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

  template<class Functor>
  void finished_queueing(CodingType ct, const Functor &put_or_get) {
    if (ct == PIP_SIGNIFICANCE_MAP) {
      bool block_of_interest = sub_mb_size >= 16; //(sub_mb_cat == 1 || sub_mb_cat == 2);
      CodingType last = coding_type;
      coding_type = PIP_SIGNIFICANCE_NZ;
      BlockMeta &meta = frames[cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y);
      int nonzero_bits[6] = {};
      for (int i = 0; i < 6; ++i) {
        nonzero_bits[i] = (meta.num_nonzeros[mb_coord.scan8_index] & (1 << i)) >> i;
      }
#define QUEUE_MODE
#ifdef QUEUE_MODE
      const uint32_t serialized_bits = sub_mb_size > 16 ? 6 : sub_mb_size > 4 ? 4 : 2;
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
          if (has_left) {
            left_nonzero_bit = (left_nonzero >= tmp);
          }
          int above_nonzero_bit = 2;
          if (above_nonzero) {
            above_nonzero_bit = (above_nonzero >= tmp);
          }
          int mb_type = frames[cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y).mb_type;
          auto *e = &queue_estimators->at(i, serialized_so_far,
                                          (frames[!cur_frame].meta_at(
                                              mb_coord.mb_x, mb_coord.mb_y).num_nonzeros[mb_coord.scan8_index] >= tmp),
                                          left_nonzero_bit,
                                          above_nonzero_bit,
                                          sub_mb_cat);
          put_or_get(e, &nonzero_bits[i], i); // FIXME: what is the appropriate context here

          if (nonzero_bits[i]) {
            serialized_so_far |= cur_bit;
          }
        } while (++i < serialized_bits);

        if (block_of_interest) {
          // LOG_NEIGHBORS("%d %d %d ", neighbor.mb_x, neighbor.mb_y, neighbor.scan8_index);
          if (sub_mb_size == 64 && mb_coord.scan8_index % 4 != 0) {
            fprintf(stderr, "%d %d\n", sub_mb_size, mb_coord.scan8_index);
            exit(1);
          }
          LOG_NEIGHBORS("%d %d %d %d ",
                        mb_coord.scan8_index, sub_mb_size, sub_mb_cat, sub_mb_is_dc);
          LOG_NEIGHBORS("<{");

          if (has_left) LOG_NEIGHBORS("%d,", left_nonzero);
          else LOG_NEIGHBORS("X,");

          if (has_above) LOG_NEIGHBORS("%d,", above_nonzero);
          else LOG_NEIGHBORS("X,");

          if (frames[!cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y).coded)
            LOG_NEIGHBORS("%d",
                          frames[!cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y).num_nonzeros[mb_coord.scan8_index]);
          else LOG_NEIGHBORS("X");
        }
      }
#endif
      meta.num_nonzeros[mb_coord.scan8_index] = 0;
      for (int i = 0; i < 6; ++i) {
        meta.num_nonzeros[mb_coord.scan8_index] |= nonzero_bits[i] << i;
      }
      if (block_of_interest) {
        LOG_NEIGHBORS("} %d> \n", meta.num_nonzeros[mb_coord.scan8_index]);
      }
      coding_type = last;
    }
  }

  void end_coding_type(CodingType ct) {
    switch (ct) {
      case PIP_SIGNIFICANCE_MAP: {
        assert(coding_type == PIP_UNREACHABLE
               || (coding_type == PIP_SIGNIFICANCE_MAP && mb_coord.zigzag_index == 0));
        uint8_t num_nonzeros = 0;
        for (int i = 0; i < sub_mb_size + (sub_mb_size == 15); ++i) {
          int16_t res = frames[cur_frame].at(mb_coord.mb_x, mb_coord.mb_y).residual[mb_coord.scan8_index * 16 + i];
          assert(res == 1 || res == 0);
          if (res != 0) {
            num_nonzeros += 1;
          }
        }
        BlockMeta &meta = frames[cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y);
        meta.is_8x8 = /*meta.is_8x8 ||*/ (sub_mb_size > 32); // 8x8 will have DC be 2x2
        meta.coded = true;
        assert(meta.num_nonzeros[mb_coord.scan8_index] == 0 || meta.num_nonzeros[mb_coord.scan8_index] == num_nonzeros);
        meta.num_nonzeros[mb_coord.scan8_index] = num_nonzeros;
        meta.sub_mb_size = sub_mb_size;
      }
        break;
      case PIP_INTRA4X4_PRED_MODE: {
        BlockMeta &meta = frames[cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y);
      }
        break;
      case PIP_MB_CBP_LUMA: {
        BlockMeta &meta = frames[cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y);
        meta.cbp_luma = cbp_luma_running;
      }
        break;
      case PIP_RESIDUALS:
        break;
      default:
        break;
    }
    coding_type = PIP_UNKNOWN;
  }

  bool begin_coding_type(CodingType ct, int zz_index, int param0, int param1) {
    bool begin_queueing = false;
    coding_type = ct;
    switch (ct) {
      case PIP_SIGNIFICANCE_MAP: {
        BlockMeta &meta = frames[cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y);
        meta.num_nonzeros[mb_coord.scan8_index] = 0;
      }
        assert(!zz_index);
        nonzeros_observed = 0;
        if (sub_mb_size == 15) {
          mb_coord.zigzag_index = 0;
        } else {
          mb_coord.zigzag_index = 0;
        }
        begin_queueing = true;
        break;
      case PIP_INTRA4X4_PRED_MODE:
        intra4x4_pred_mode_bit_num = 0;
        intra4x4_pred_mode_last = 0;
        intra4x4_pred_mode_running = 0;
        intra4x4_pred_mode_last_pred = param0;
        break;
      case PIP_MB_CBP_LUMA:
        cbp_luma_bit_num = 0;
        cbp_luma_last = 0;
        cbp_luma_running = 0;
        break;
      case PIP_MB_MVD:
        mvd_ind = param0;
        mvd_bit_num = 0;
        mvd_state = 0;
        break;
      case PIP_RESIDUALS:
        residual_index = param0;
        break;
      case PIP_CODED_BLOCK:
        cbf_nza = param0;
        cbf_nzb = param1;
        break;
      case PIP_INTRA_MB_TYPE:
        intra_mb_left = parse_ff_mb_type(param0);
        intra_mb_top = parse_ff_mb_type(param1);
        intra_slice = zz_index; // ugh
        intra_mb_bit_num = 0;
        intra_mb_cbp_chroma = false;
        break;
      default:
        break;
    }
    return begin_queueing;
  }

  void reset_mb_significance_state_tracking() {
    mb_coord.zigzag_index = 0;
    nonzeros_observed = 0;
    coding_type = PIP_SIGNIFICANCE_MAP;
  }

  void update_state_tracking(int symbol, int context) {
    switch (coding_type) {
      case PIP_SIGNIFICANCE_NZ:
      case PIP_MB_SKIP_FLAG:
      case PIP_MB_CHROMA_PRE_MODE:
      case PIP_MB_CBP_CHROMA:
      case PIP_P_MB_SUB_TYPE:
      case PIP_B_MB_SUB_TYPE:
      case PIP_MB_REF:
      case PIP_CODED_BLOCK:
        break;
      case PIP_INTRA_MB_TYPE:
        if (intra_mb_bit_num == 2)
          intra_mb_cbp_chroma = symbol;
        if (context != terminate_context)
          intra_mb_bit_num++;
        break;
      case PIP_MB_MVD:
        if (context == bypass_context && mvd_state == 0) {
          mvd_state++;
          mvd_bit_num = -1;
        }
        if (context == bypass_context && mvd_state == 1 && symbol == 0) {
          mvd_state++;
          mvd_bit_num = -1;
        }
        mvd_bit_num++;
        break;
      case PIP_MB_CBP_LUMA:
        cbp_luma_last = symbol;
        cbp_luma_running += symbol << cbp_luma_bit_num;
        cbp_luma_bit_num++;
        break;
      case PIP_INTRA4X4_PRED_MODE:
        if (intra4x4_pred_mode_bit_num) {
          intra4x4_pred_mode_running <<= 1;
          intra4x4_pred_mode_running |= symbol;
        }
        intra4x4_pred_mode_bit_num++;
        intra4x4_pred_mode_last = symbol;
        break;
      case PIP_SIGNIFICANCE_MAP:
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
              // if we were a zero and we haven't eob'd then the
              // next and last must be a one
              frames[cur_frame].at(mb_coord.mb_x, mb_coord.mb_y).residual[mb_coord.scan8_index * 16 +
                                                                          mb_coord.zigzag_index] = 1;
              ++nonzeros_observed;
              coding_type = PIP_UNREACHABLE;
              mb_coord.zigzag_index = 0;
            }
          }
        }
        break;
      case PIP_SIGNIFICANCE_EOB:
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
        break;
      case PIP_RESIDUALS:
        residuals_bit_num++;
        if (context == sign_bypass_context) {
          residuals_bit_num = 0;
        }
        break;
      case PIP_UNKNOWN:
        break;
      case PIP_UNREACHABLE:
        assert(false);
      default:
        assert(false);
    }
  }

  void update_state(int symbol, int context) {
    auto *e = get_estimator(context, symbol);
    update_state_for_model_key(symbol, context, e);
  }

  void update_state_for_model_key(int symbol, int context, estimator* e) {
    if (coding_type == PIP_SIGNIFICANCE_EOB) {
      int num_nonzeros = frames[cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y).num_nonzeros[mb_coord.scan8_index];
      assert(symbol == (num_nonzeros == nonzeros_observed));
    }
    if (symbol) {
      e->pos++;
    } else {
      e->neg++;
    }

    switch (coding_type) {
      case PIP_RESIDUALS:
      case PIP_MB_MVD:
      case PIP_MB_CBP_LUMA:
        if (e->pos + e->neg > 512)
          e->renormalize();
        break;
      case PIP_SIGNIFICANCE_MAP:
        if (e->pos + e->neg > 512)
          e->renormalize();
        break;
      case PIP_SIGNIFICANCE_NZ:
        if (e->pos + e->neg > 800)
          e->renormalize();
        break;
      default:
        if (e->pos + e->neg > 0xA0)
          e-> renormalize();
        break;
    }
    update_state_tracking(symbol, context);
  }

  void copy_coefficients(int16_t *block, int max_coeff) {
    int copy_size = max_coeff + (max_coeff == 15);
    if (max_coeff == 4) copy_size = 64;  // TODO: ffmpeg does something weird with chroma DCs
    memcpy(&frames[cur_frame].at(mb_coord.mb_x, mb_coord.mb_y).
              coeffs[mb_coord.scan8_index * 16],
           block,
           copy_size * sizeof(int16_t));
  }

  void print_coeffs() {
    return;

    int16_t *block = &frames[cur_frame].at(mb_coord.mb_x, mb_coord.mb_y).
        coeffs[mb_coord.scan8_index * 16];

    if (frames[cur_frame].get_frame_num() > 1) return;
    /*const uint8_t *unscan = (sub_mb_size > 4) ? ffmpeg_j_to_ind : zigzag4;
    unscan += (sub_mb_size == 15);
    for (int j = 0; j < sub_mb_size; j++) {
      int ind = unscan[j];
      if (block[j]) printf("%d,%d,%d,%d,%d,%d,%d\n",
                           frames[cur_frame].get_frame_num(),
                           mb_coord.mb_x, mb_coord.mb_y, mb_coord.scan8_index,
                           ind, j, block[j]);
    }*/
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
    }
  }

  void set_mb_type(int ff_mb_type) {
    BlockMeta &meta = frames[cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y);
    meta.mb_type = parse_ff_mb_type(ff_mb_type);
  }

 private:
  // TODO: DELETE THIS
  estimator* get_estimator_helper(int context) {
    switch (context) {
      case bypass_context:
        return &bypass_estimator;
      case sign_bypass_context:
        return &sign_bypass_esimator;
      case terminate_context:
        return &terminate_estimator;
      case eob_context:
        return &eob_estimator[eob_symbol()];
      default:
        if (context >= 0 && context < CABAC_STATE_SIZE) {
          return &cabac_estimator[context];
        } else {
          fprintf(stderr, "UHOH %d\n", context);
          assert(false);
        }
    }
  }

  estimator* get_estimator(int context, int symbol) {
    switch (coding_type) {
      case PIP_SIGNIFICANCE_MAP: {
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
        if (sub_mb_is_dc && sub_mb_chroma422) {
          assert(mb_coord.zigzag_index < 7);
          zigzag_offset = sig_coeff_offset_dc[mb_coord.zigzag_index];
        } else {
          if (sub_mb_size > 32) {
            assert(mb_coord.zigzag_index < 63);
            zigzag_offset = sig_coeff_flag_offset_8x8[0][mb_coord.zigzag_index];
          }
        }
        assert(sub_mb_cat < 13);  // FIXME: although this let's us get rid of a table
        int neighbor_above = 0;
        int neighbor_left = 0;
        int coeff_neighbor_above = 0;
        int coeff_neighbor_left = 0;
        do_print = false;
        if (do_print) {
          LOG_NEIGHBORS("[");
        }
        {
          CoefficientCoord neighbor_left_coord = {0, 0, 0, 0};
          if (get_neighbor(false, sub_mb_size, mb_coord, &neighbor_left_coord)) {
            int16_t tmp = 0;
            const bool matched =
                frames[cur_frame].meta_at(neighbor_left_coord.mb_x, neighbor_left_coord.mb_y).sub_mb_size == sub_mb_size;
            if (fetch(false, true, neighbor_left_coord, &tmp) && matched) {
              neighbor_left = 4 + std::max(std::min(tmp, (int16_t) 2), (int16_t) -2);
              if (do_print) {
                LOG_NEIGHBORS("%d,", tmp);
              }
            } else {
              neighbor_left = 1;
              if (do_print) {
                LOG_NEIGHBORS("_,");
              }
            }

            /*flip = !flip;
            if (TOTAL_NUM < 10000 && flip) {
              LOG_NEIGHBORS("%d %d %d\n", sub_mb_is_dc, sub_mb_cat, sub_mb_size);
              LOG_NEIGHBORS("%d %d %d %d\n",
                            mb_coord.mb_x, mb_coord.mb_y,
                            mb_coord.scan8_index, mb_coord.zigzag_index);
              LOG_NEIGHBORS("%d %d %d %d\n\n",
                            neighbor_left_coord.mb_x, neighbor_left_coord.mb_y,
                            neighbor_left_coord.scan8_index, neighbor_left_coord.zigzag_index);
            }*/

          } else {
            if (do_print) {
              LOG_NEIGHBORS("x,");
            }
          }
        }
        /*{
          CoefficientCoord neighbor_above_coord = {0, 0, 0, 0};
          if (get_neighbor(true, sub_mb_size, mb_coord, &neighbor_above_coord)) {
            int16_t tmp = 0;
            if (fetch(false, true, neighbor_above_coord, &tmp)) {
              neighbor_above = 2 + tmp;
              if (do_print) {
                LOG_NEIGHBORS("%d,", tmp);
              }
            } else {
              neighbor_above = 1;
              if (do_print) {
                LOG_NEIGHBORS("_,");
              }
            }
          } else {
            if (do_print) {
              LOG_NEIGHBORS("x,");
            }
          }
        }
        {
          CoefficientCoord neighbor_left_coord = {0, 0, 0, 0};
          if (get_neighbor_coefficient(false, sub_mb_size, mb_coord, &neighbor_left_coord)) {
            int16_t tmp = 0;
            if (fetch(false, true, neighbor_left_coord, &tmp)) {
              coeff_neighbor_left = 2 + !!tmp;
            } else {
              coeff_neighbor_left = 1;
            }
          } else {
          }
        }
        {
          CoefficientCoord neighbor_above_coord = {0, 0, 0, 0};
          if (get_neighbor_coefficient(true, sub_mb_size, mb_coord, &neighbor_above_coord)) {
            int16_t tmp = 0;
            if (fetch(false, true, neighbor_above_coord, &tmp)) {
              coeff_neighbor_above = 2 + !!tmp;
            } else {
              coeff_neighbor_above = 1;
            }
          } else {
          }
        }*/

        int coeff_prev = 0;
        // FIXME: why doesn't this prior help at all
        {
          int16_t output = 0;
          if (fetch(true, true, mb_coord, &output)) {
            if (do_print) LOG_NEIGHBORS("%d] ", output);
            // coeff_prev = 4 + std::max(std::min(output, (int16_t) 3), (int16_t) -3);
            coeff_prev = 3 + std::max(std::min(output, (int16_t) 1), (int16_t) -1);
          } else {
            if (do_print) LOG_NEIGHBORS("x] ");
            coeff_prev = 1;
          }
        }
        int num_nonzeros = frames[cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y).num_nonzeros[mb_coord.scan8_index];
        /*if (COUNT_TOTAL_SYMBOLS < NUM_SYMBOLS_PRINT)
          fprintf(LOG_OUT, "%d,%d,%d,%d,%d\n",
                  mb_coord.scan8_index,
                  nonzeros_observed, num_nonzeros - nonzeros_observed,
                  zigzag_offset, sub_mb_cat);*/

        int mb_type = frames[cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y).mb_type;
        auto *e = &significance_estimator->at(num_nonzeros - nonzeros_observed,
                                              zigzag_offset,
                                              sub_mb_cat,
                                              mb_type, 0, 0, 0);
        return e;
      }
      // FIXME: why doesn't this prior help at all
      case PIP_SIGNIFICANCE_EOB:
        return &eob_estimator[eob_symbol()];
      case PIP_INTRA4X4_PRED_MODE: {
        if (!intra4x4_pred_mode_bit_num)
          return &intra4x4_pred_mode_skip[intra4x4_pred_mode_last_pred];
        else
          return &intra4x4_pred_mode_estimator[intra4x4_pred_mode_bit_num - 1]
              [intra4x4_pred_mode_running][intra4x4_pred_mode_last_pred];
      }
      case PIP_MB_CBP_LUMA: {
        int left = mb_coord.mb_x != 0;
        if (left) left += frames[cur_frame].meta_at(mb_coord.mb_x - 1, mb_coord.mb_y).cbp_luma;
        int top = mb_coord.mb_y != 0;
        if (top) top += frames[cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y - 1).cbp_luma;
        int last = frames[cur_frame].get_frame_num() != 0;
        if (last) last += frames[!cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y).cbp_luma;
        // losslessh264 uses the block type, which we do not currently track.
        int mb_type = frames[cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y).mb_type;
        int prev_mb_type = frames[!cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y).mb_type;

        return get_estimator_helper(context);

        /*if (mb_type == prev_mb_type)
          return &mb_cbp_luma[last][mb_type][cbp_luma_bit_num];
        else
          return get_estimator_helper(context);*/
      }
      case PIP_MB_SKIP_FLAG: {
        int left = mb_coord.mb_x != 0;
        if (left) left += frames[cur_frame].meta_at(mb_coord.mb_x - 1, mb_coord.mb_y).coded;
        int top = mb_coord.mb_y != 0;
        if (top) top += frames[cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y - 1).coded;
        int last = frames[cur_frame].get_frame_num() != 0;
        if (last) last += frames[!cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y).coded;
        return &mb_skip_estimator[left][top][last];
      }
      case PIP_MB_MVD: {
        int mb_type = frames[cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y).mb_type;
        if (context == sign_bypass_context) {
          return &mb_mvd_sign_est[0][0];
        } else if (context == bypass_context) {
          int clipped_bit_num = std::min(mvd_bit_num, 9);
          return &mb_mvd_exp_est[mvd_state - 1][mvd_bit_num];
        } else {
          int clipped_num = std::min(mvd_bit_num, 10);
          return &mb_mvd_estimator[mvd_ind][clipped_num][mb_type];
        }
      }
      case PIP_RESIDUALS: {
        if (context == sign_bypass_context) {
          return &residuals_sign_est[sub_mb_cat][residual_index];
        } else if (context == bypass_context) {
          return &residuals_bypass_est;
        } else {
          if (!residuals_bit_num) {
            return &residuals_is_one_est[sub_mb_cat][residual_index];
          }
          int tmp = std::min(residuals_bit_num - 1, 16);
          return &residuals_est->at(sub_mb_cat, residual_index, tmp);
        }
      }
      case PIP_CODED_BLOCK: {
        return &coded_block_est[sub_mb_cat][mb_coord.scan8_index]
            [!!cbf_nza][!!cbf_nzb];
      }
        break;
      case PIP_INTRA_MB_TYPE: {
        if (context == terminate_context) {
          nearly_zero.pos = 1;
          nearly_zero.neg = 2047;
          return &nearly_zero;
        }
        return &intra_mb_type_est[intra_mb_left][intra_mb_top][intra_slice][intra_mb_bit_num][intra_mb_cbp_chroma];
      }
      case PIP_SIGNIFICANCE_NZ:
      case PIP_UNREACHABLE:
      case PIP_MB_CHROMA_PRE_MODE:
      case PIP_MB_CBP_CHROMA:
      case PIP_P_MB_SUB_TYPE:
      case PIP_B_MB_SUB_TYPE:
      case PIP_MB_REF:
        return get_estimator_helper(context);
      case PIP_UNKNOWN:
        return get_estimator_helper(context);
      default:
        break;
    }
    assert(false && "Unreachable");
    abort();
  }
};

constexpr int h264_model::bypass_context, h264_model::terminate_context;
constexpr int h264_model::significance_context, h264_model::eob_context;
constexpr int h264_model::sign_bypass_context;
