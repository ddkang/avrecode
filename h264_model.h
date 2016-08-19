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
#include "cavlc_estimators.h"

#pragma once


#define STRINGIFY_COMMA(s) #s ,
const char * billing_names [] = {EACH_PIP_CODING_TYPE(STRINGIFY_COMMA)};
#undef STRINGIFY_COMMA

class h264_model {
 public:
  CodingType coding_type = PIP_UNKNOWN;

  h264_model() {
    EstimatorContext::reset();
    reset();
    memset(bill, 0, sizeof(bill));
    memset(cabac_bill, 0, sizeof(cabac_bill));
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

  void update_state(int symbol, int context) {
    auto *e = get_estimator(context);
    update_state_for_model_key(symbol, context, e);
  }

  void update_state_tracking(int symbol, int context) {
    if (coding_type == PIP_UNREACHABLE)
      assert(false);
    // Annoyingly, this is a special case
    if (coding_type == PIP_SIGNIFICANCE_NZ && all_estimators[coding_type] == nullptr)
      return;
    coding_type = all_estimators[coding_type]->update(symbol, context);
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

  // template<class Functor>
  virtual void finished_queueing(CodingType ct, const std::function<void(estimator *, int *, int)> &put_or_get) = 0;
  virtual void end_coding_type(CodingType ct) = 0;
  virtual bool begin_coding_type(CodingType ct, int zz_index, int param0, int param1) = 0;
  virtual void reset_mb_significance_state_tracking() = 0;


 protected:
  std::array<EstimatorContext *, PIP_NUM_TYPES> all_estimators;

  estimator* get_estimator(int context) {
    if (coding_type == PIP_UNREACHABLE) {
      assert(false && "Unreachable");
      abort();
    }
    return all_estimators[coding_type]->get_estimator(context);
  }


 private:
  size_t bill[sizeof(billing_names) / sizeof(billing_names[0])];
  size_t cabac_bill[sizeof(billing_names) / sizeof(billing_names[0])];
};

class cabac_model : public h264_model {
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
  cabac_model() {
    // Ugh
    all_estimators[PIP_UNKNOWN] = &generic_est;
    all_estimators[PIP_SIGNIFICANCE_MAP] = &sig_map_est;
    all_estimators[PIP_SIGNIFICANCE_EOB] = &eob_est;
    all_estimators[PIP_RESIDUALS] = &residuals;
    all_estimators[PIP_INTRA_MB_TYPE] = &intra_mb_type;
    all_estimators[PIP_INTRA4X4_PRED_MODE] = &intra4x4_pred;
    all_estimators[PIP_MB_MVD] = &mvd_est;
    all_estimators[PIP_MB_SKIP_FLAG] = &mb_skip;
    all_estimators[PIP_MB_CHROMA_PRE_MODE] = &chroma_pre_mode;
    all_estimators[PIP_MB_CBP_CHROMA] = &cbp_chroma;
    all_estimators[PIP_CODED_BLOCK] = &coded_block;
    // Not worth on iphone
    all_estimators[PIP_P_MB_SUB_TYPE] = &generic_est;
    all_estimators[PIP_B_MB_SUB_TYPE] = &generic_est;
    all_estimators[PIP_MB_REF] = &generic_est;
    // Maybe worth, but I haven't figured out the right priors
    all_estimators[PIP_MB_CBP_LUMA] = &generic_est;
    // Should never be used in CABAC
    all_estimators[PIP_UNREACHABLE] = nullptr;
    all_estimators[PIP_SIGNIFICANCE_NZ] = nullptr;
  }

  void finished_queueing(CodingType ct, const std::function<void(estimator *, int *, int)> &put_or_get) {
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
    bool begin_queueing = ct == PIP_SIGNIFICANCE_MAP;
    coding_type = ct;
    all_estimators[coding_type]->begin(zz_index, param0, param1);
    return begin_queueing;
  }

  void reset_mb_significance_state_tracking() {
    sig_map_est.reset_mb_significance_state_tracking();
    coding_type = PIP_SIGNIFICANCE_MAP;
  }
};

class cavlc_model : public h264_model {
 private:
  // FIXME
  CABACGenericEst generic_est;

 public:
  cavlc_model() {
    all_estimators[PIP_UNKNOWN] = &generic_est;
    all_estimators[PIP_RESIDUALS] = &residuals_est;
    all_estimators[PIP_INTRA_MB_TYPE] = &mb_type_est;
    all_estimators[PIP_INTRA4X4_PRED_MODE] = &intra4x4_pred_mode_est;
    all_estimators[PIP_MB_CBP_LUMA] = &cbp_est;
    all_estimators[PIP_MB_MVD] = &mvd_est;
    all_estimators[PIP_MB_SKIP_FLAG] = &skip_est;
    all_estimators[PIP_B_MB_SUB_TYPE] = &sub_mb_b_est;
    all_estimators[PIP_P_MB_SUB_TYPE] = &sub_mb_p_est;
    all_estimators[PIP_CHROMA_PRED_MODE] = &chroma_pred_mode_est;
    all_estimators[PIP_QUANT_DELTA] = &quant_delta_est;

    all_estimators[PIP_RUN_BEFORE] = &run_before_est;
    all_estimators[PIP_ZEROS_LEFT] = &zeros_left_est;
    all_estimators[PIP_REMAINING_LEVEL_CODE] = &remaining_level_code_est;
    all_estimators[PIP_REMAINING_LEVEL] = &remaining_level_est;
    all_estimators[PIP_FIRST_LEVEL_CODE] = &first_level_code_est;
    all_estimators[PIP_FIRST_LEVEL] = &first_level_est;
    all_estimators[PIP_LEVEL_SETUP] = &level_setup_est;
    all_estimators[PIP_COEFF_TOKEN_CHROMA] = &coef_token_chroma_est;
    all_estimators[PIP_COEFF_TOKEN] = &coeff_token_est;
  }

  void finished_queueing(CodingType ct, const std::function<void(estimator *, int *, int)> &put_or_get) {

  }

  void end_coding_type(CodingType ct) {
    coding_type = PIP_UNKNOWN;
  }

  bool begin_coding_type(CodingType ct, int zz_index, int param0, int param1) {
    coding_type = ct;
    all_estimators[coding_type]->begin(zz_index, param0, param1);
    return false;
  }

  void reset_mb_significance_state_tracking() {

  }

 private:
  CAVLCMvdEst mvd_est;
  CAVLCIntra4x4PredModeEst intra4x4_pred_mode_est;
  CAVLCCbpEst cbp_est;
  CAVLCMbSkipEst skip_est;
  CAVLCMbTypeEst mb_type_est;
  CAVLCChromaPredModeEst chroma_pred_mode_est;
  CAVLCBSubMbTypeEst sub_mb_b_est;
  CAVLCPSubMbTypeEst sub_mb_p_est;
  CAVLCQuantDeltaEst quant_delta_est;

  CAVLCResidualsEst residuals_est;
  CAVLCRunBeforeEst run_before_est;
  CAVLCZerosLeftEst zeros_left_est;
  CAVLCRemainingLevelCodeEst remaining_level_code_est;
  CAVLCRemainingLevelEst remaining_level_est;
  CAVLCFirstLevelCodeEst first_level_code_est;
  CAVLCFirstLevelEst first_level_est;
  CAVLCLevelSetupEst level_setup_est;
  CAVLCCoeffTokenChromaEst coef_token_chroma_est;
  CAVLCCoeffTokenEst coeff_token_est;
};