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
#include "estimators.h"

#pragma once


#define STRINGIFY_COMMA(s) #s ,
const char * billing_names [] = {EACH_PIP_CODING_TYPE(STRINGIFY_COMMA)};
#undef STRINGIFY_COMMA

class h264_model {
 private:
  CABACGenericEst generic_est;

  CABACChromaPreModeEst chroma_pre_mode;
  CABACCodedBlockEst coded_block;
  CABACIntraMBTypeEst intra_mb_type;
  CABACCbpChromaEst cbp_chroma;
  CABACCbpLumaEst cbp_luma;
  CABACIntra4x4PredEst intra4x4_pred;
  CABACMBSkipEst mb_skip;

  CABACMBMvdEst mvd_est;
  CABACResidualsEst residuals;
  CABACSignificanceMapEst sig_map_est;
  CABACSignificanceEOBEst eob_est;

 public:
  size_t bill[sizeof(billing_names) / sizeof(billing_names[0])];
  size_t cabac_bill[sizeof(billing_names) / sizeof(billing_names[0])];

  // Global context
  CodingType coding_type = PIP_UNKNOWN;
  bool do_print;

 public:
  h264_model() {
    EstimatorContext::reset();
    reset();
    do_print = false;
    memset(bill, 0, sizeof(bill));
    memset(cabac_bill, 0, sizeof(cabac_bill));
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

  range_t probability_for_model_key(range_t range, estimator *e) {
    int total = e->pos + e->neg;
    return (range / total) * e->pos;
  }

  range_t probability_for_state(range_t range, int context) {
    auto *e = get_estimator(context);
    return probability_for_model_key(range, e);
  }

  template<class Functor>
  void finished_queueing(CodingType ct, const Functor &put_or_get) {
    if (ct == PIP_SIGNIFICANCE_MAP) {
      coding_type = PIP_SIGNIFICANCE_NZ;
      sig_map_est.finish_queueing(put_or_get);
      coding_type = PIP_SIGNIFICANCE_MAP;
    }
  }

  void end_coding_type(CodingType ct) {
    switch (ct) {
      case PIP_SIGNIFICANCE_MAP:
        sig_map_est.end_coding_type(coding_type);
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
      case PIP_SIGNIFICANCE_MAP:
        sig_map_est.begin(zz_index, param0, param1);
        begin_queueing = true;
        break;
      case PIP_INTRA4X4_PRED_MODE:
        intra4x4_pred.begin(zz_index, param0, param1);
        break;
      case PIP_MB_CBP_LUMA:
        cbp_luma.begin(zz_index, param0, param1);
        break;
      case PIP_MB_MVD:
        mvd_est.begin(zz_index, param0, param1);
        break;
      case PIP_RESIDUALS:
        residuals.begin(zz_index, param0, param1);
        break;
      case PIP_CODED_BLOCK:
        coded_block.begin(zz_index, param0, param1);
        break;
      case PIP_INTRA_MB_TYPE:
        intra_mb_type.begin(zz_index, param0, param1);
        break;
      case PIP_MB_CBP_CHROMA:
        cbp_chroma.begin(zz_index, param0, param1);
        break;
      case PIP_MB_CHROMA_PRE_MODE:
        chroma_pre_mode.begin(zz_index, param0, param1);
        break;
      case PIP_MB_SKIP_FLAG:
        mb_skip.begin(zz_index, param0, param1);
        break;
      default:
        break;
    }
    return begin_queueing;
  }

  void reset_mb_significance_state_tracking() {
    sig_map_est.reset_mb_significance_state_tracking();
    coding_type = PIP_SIGNIFICANCE_MAP;
  }

  void update_state_tracking(int symbol, int context) {
    switch (coding_type) {
      case PIP_SIGNIFICANCE_NZ:
      case PIP_MB_SKIP_FLAG:
      case PIP_P_MB_SUB_TYPE:
      case PIP_B_MB_SUB_TYPE:
      case PIP_MB_REF:
      case PIP_CODED_BLOCK:
        break;
      case PIP_MB_CHROMA_PRE_MODE:
        chroma_pre_mode.update(symbol, context);
        break;
      case PIP_MB_CBP_CHROMA:
        cbp_chroma.update(symbol, context);
        break;
      case PIP_INTRA_MB_TYPE:
        intra_mb_type.update(symbol, context);
        break;
      case PIP_MB_MVD:
        mvd_est.update(symbol, context);
        break;
      case PIP_MB_CBP_LUMA:
        cbp_luma.update(symbol, context);
        break;
      case PIP_INTRA4X4_PRED_MODE:
        intra4x4_pred.update(symbol, context);
        break;
      case PIP_SIGNIFICANCE_MAP:
        coding_type = sig_map_est.update(symbol, context);
        break;
      case PIP_SIGNIFICANCE_EOB:
        coding_type = eob_est.update(symbol, context);
        break;
      case PIP_RESIDUALS:
        residuals.update(symbol, context);
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
    auto *e = get_estimator(context);
    update_state_for_model_key(symbol, context, e);
  }

  void update_state_for_model_key(int symbol, int context, estimator* e) {
    if (coding_type == PIP_SIGNIFICANCE_EOB)
      assert(symbol == EstimatorContext::eob_symbol());
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

 private:
  estimator* get_estimator(int context) {
    switch (coding_type) {
      case PIP_SIGNIFICANCE_MAP:
        return sig_map_est.get_estimator(context);
      // FIXME: why doesn't this prior help at all
      case PIP_SIGNIFICANCE_EOB:
        return eob_est.get_estimator(context);
      case PIP_INTRA4X4_PRED_MODE:
        return intra4x4_pred.get_estimator(context);
      case PIP_MB_CBP_LUMA: {
        /*int left = mb_coord.mb_x != 0;
        if (left) left += frames[cur_frame].meta_at(mb_coord.mb_x - 1, mb_coord.mb_y).cbp_luma;
        int top = mb_coord.mb_y != 0;
        if (top) top += frames[cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y - 1).cbp_luma;
        int last = frames[cur_frame].get_frame_num() != 0;
        if (last) last += frames[!cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y).cbp_luma;*/
        // losslessh264 uses the block type, which we do not currently track.
        // int mb_type = frames[cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y).mb_type;
        // int prev_mb_type = frames[!cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y).mb_type;

        return generic_est.get_estimator(context);
        // return cbp_luma.get_estimator(context);

        /*if (mb_type == prev_mb_type)
          return &mb_cbp_luma[last][mb_type][cbp_luma_bit_num];
        else
          return generic_est.get_estimator(context);*/
      }
      case PIP_MB_SKIP_FLAG:
        return  mb_skip.get_estimator(context);
      case PIP_MB_MVD:
        return mvd_est.get_estimator(context);
      case PIP_RESIDUALS:
        return residuals.get_estimator(context);
      case PIP_CODED_BLOCK:
        return coded_block.get_estimator(context);
      case PIP_INTRA_MB_TYPE:
        return intra_mb_type.get_estimator(context);
      case PIP_MB_CBP_CHROMA:
        return cbp_chroma.get_estimator(context);
      case PIP_MB_CHROMA_PRE_MODE:
        return chroma_pre_mode.get_estimator(context);
      case PIP_SIGNIFICANCE_NZ:
      case PIP_UNREACHABLE:
      // These are definitely not worth it on iphone
      case PIP_P_MB_SUB_TYPE:
      case PIP_B_MB_SUB_TYPE:
      case PIP_MB_REF:
        return generic_est.get_estimator(context);
      case PIP_UNKNOWN:
        return generic_est.get_estimator(context);
      default:
        break;
    }
    assert(false && "Unreachable");
    abort();
  }
};