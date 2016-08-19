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

class UnaryEstimator : public EstimatorContext {
 public:
  void begin(const int zz_index, const int param0, const int param1) {
    bit_num = 0;
  }

  // This class should never really be used on its face, so w/e
  CodingType update(const int symbol, const int context) {
    bit_num++;
    return PIP_UNKNOWN;
  }

  estimator* get_estimator(const int context) {
    return &est[std::min(bit_num, MAX_NUM - 1)];
  }

  int bit_num = 0;
 protected:
  static constexpr int MAX_NUM = 10;
  estimator est[MAX_NUM];
};

class GolombEstimator : public EstimatorContext {
 public:
  void begin(const int zz_index, const int param0, const int param1) {
    state = UNARY;
    unary_est.begin(zz_index, param0, param1);
    mantissa_est.begin(zz_index, param0, param1);
  }

  // This class should never really be used on its face, so w/e
  CodingType update(const int symbol, const int context) {
    switch (state) {
      case UNARY:
        if (symbol) state = MANTISSA;
        return unary_est.update(symbol, context);
      case MANTISSA:
        if (unary_est.bit_num == mantissa_est.bit_num) {
          auto e = mantissa_est.update(symbol, context);
          begin(0, 0, 0);
          return e;
        } else
          return mantissa_est.update(symbol, context);
      case DONE:
        begin(0, 0, 0);
        return PIP_UNKNOWN;
    }
  }

  estimator* get_estimator(const int context) {
    switch (state) {
      case UNARY:
        return unary_est.get_estimator(context);
      case MANTISSA:
        return mantissa_est.get_estimator(context);
      case DONE:
        assert(false);
    }
  }

 private:
  enum STATE {
    UNARY,
    MANTISSA,
    DONE
  };
  STATE state;
  UnaryEstimator unary_est, mantissa_est;
};

class CAVLCMvdEst : public EstimatorContext {
 public:
  void begin(const int zz_index, const int param0, const int param1) {
    index = param0;
    is_x = param1;
    est[mb_type][index][is_x].begin(zz_index, param0, param1);
  }

  CodingType update(const int symbol, const int context) {
    est[mb_type][index][is_x].update(symbol, context);
    return PIP_MB_MVD;
  }

  estimator* get_estimator(const int context) {
    return est[mb_type][index][is_x].get_estimator(context);
  }

 private:
  // FIXME: this should actually be a signed golomb code
  GolombEstimator est[MB_NUM_TYPES][16][2];
  int index;
  bool is_x;
};

class CAVLCIntra4x4PredModeEst : public EstimatorContext {
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
  int bit_num, mode, index, running;
  estimator skip_est[9];
  estimator est[16][9][9][3];
};

// This only gets ~10% compared to the claimed 20% of losslessh264
// losslessh264 uses the past cbp_luma which we do not track.
class CAVLCCbpEst : public EstimatorContext {
 public:
  void begin(const int zz_index, const int param0, const int param1) {
    decode_chroma = param0;
    est[mb_type][decode_chroma].begin(zz_index, param0, param1);
  }

  CodingType update(const int symbol, const int context) {
    est[mb_type][decode_chroma].update(symbol, context);
    return PIP_MB_CBP_LUMA;
  }

  estimator* get_estimator(const int context) {
    return est[mb_type][decode_chroma].get_estimator(context);
  }

 private:
  bool decode_chroma;
  GolombEstimator est[MB_NUM_TYPES][2];
  estimator blank;
};

class CAVLCMbSkipEst : public EstimatorContext {
 public:
  void begin(const int zz_index, const int param0, const int param1) {
    slice_type = param0;
    est[slice_type].begin(zz_index, param0, param1);
  }

  CodingType update(const int symbol, const int context) {
    est[slice_type].update(symbol, context);
    return PIP_MB_SKIP_FLAG;
  }

  estimator* get_estimator(const int context) {
    return est[slice_type].get_estimator(context);
  }

 private:
  int slice_type = 0;
  GolombEstimator est[8];
};

// Only gets ~18% compared to the ~35% losslessh264 gets
// Keep track of past
class CAVLCMbTypeEst : public EstimatorContext {
 public:
  void begin(const int zz_index, const int param0, const int param1) {
    slice_type = param0;
    est[slice_type].begin(zz_index, param0, param1);
  }

  CodingType update(const int symbol, const int context) {
    est[slice_type].update(symbol, context);
    return PIP_INTRA_MB_TYPE;
  }

  estimator* get_estimator(const int context) {
    return est[slice_type].get_estimator(context);
  }

 private:
  int slice_type = 0;
  GolombEstimator est[8];
};

class CAVLCChromaPredModeEst : public EstimatorContext {
 public:
  void begin(const int zz_index, const int param0, const int param1) {
    est[mb_type].begin(zz_index, param0, param1);
  }

  CodingType update(const int symbol, const int context) {
    est[mb_type].update(symbol, context);
    return PIP_CHROMA_PRED_MODE;
  }

  estimator* get_estimator(const int context) {
    return est[mb_type].get_estimator(context);
  }

 private:
  GolombEstimator est[MB_NUM_TYPES];
};

// Not used in walk, but I'll keep it here for bookkeeping purposes.
class CAVLCBSubMbTypeEst : public CABACGenericEst {
  CodingType update(const int symbol, const int context) {
    return PIP_B_MB_SUB_TYPE;
  }
};

class CAVLCPSubMbTypeEst : public EstimatorContext {
 public:
  void begin(const int zz_index, const int param0, const int param1) {
    index = param0;
    est.begin(zz_index, param0, param1);
  }

  CodingType update(const int symbol, const int context) {
    est.update(symbol, context);
    return PIP_P_MB_SUB_TYPE;
  }

  estimator* get_estimator(const int context) {
    return est.get_estimator(context);
  }

 private:
  int index = 0;
  GolombEstimator est;
};

class CAVLCQuantDeltaEst : public EstimatorContext {
 public:
  void begin(const int zz_index, const int param0, const int param1) {
    est[mb_type].begin(zz_index, param0, param1);
  }

  CodingType update(const int symbol, const int context) {
    est[mb_type].update(symbol, context);
    return PIP_QUANT_DELTA;
  }

  estimator* get_estimator(const int context) {
    return est[mb_type].get_estimator(context);
  }

 private:
  GolombEstimator est[MB_NUM_TYPES];
};

// Just for bookkeeping for now
class CAVLCResidualsEst : public CABACGenericEst {
  CodingType update(const int symbol, const int context) {
    return PIP_RESIDUALS;
  }
};
class CAVLCRunBeforeEst : public EstimatorContext {
 public:
  void begin(const int zz_index, const int param0, const int param1) {
    zeros_left = param0;
    index = param1;
    bit_num = 0;
  }
  CodingType update(const int symbol, const int context) {
    bit_num++;
    return PIP_RUN_BEFORE;
  }
  estimator* get_estimator(const int context) {
    return &est[zeros_left][index][bit_num][sub_mb_cat];
  }

 private:
  int zeros_left, index, bit_num;
  estimator est[16][16][12][6];
};

class CAVLCZerosLeftEst : public EstimatorContext {
 public:
  void begin(const int zz_index, const int param0, const int param1) {
    total_coeff = param0;
    bit_num = 0;
  }
  CodingType update(const int symbol, const int context) {
    bit_num++;
    return PIP_ZEROS_LEFT;
  }
  estimator* get_estimator(const int context) {
    return &est[sub_mb_cat][total_coeff][bit_num];
  }

 private:
  int total_coeff, bit_num;
  estimator est[6][16][9];
};

class CAVLCFirstLevelCodeEst : public EstimatorContext {
 public:
  void begin(const int zz_index, const int param0, const int param1) {
    bit_num = 0;
    suffix_length = zz_index;
    total_coeff = param0;
    trailing_ones = param1;
  }
  CodingType update(const int symbol, const int context) {
    bit_num++;
    return PIP_FIRST_LEVEL_CODE;
  }
  estimator* get_estimator(const int context) {
    return &est[total_coeff][trailing_ones][bit_num][sub_mb_cat];
  }


 private:
  int suffix_length, total_coeff, trailing_ones;
  int bit_num;
  estimator est[64][64][8][6];
};

class CAVLCRemainingLevelCodeEst : public EstimatorContext {
 public:
  void begin(const int zz_index, const int param0, const int param1) {
    bit_num = 0;
    index = param0;
    suffix_length = param1;
  }
  CodingType update(const int symbol, const int context) {
    bit_num++;
    return PIP_REMAINING_LEVEL_CODE;
  }
  estimator* get_estimator(const int context) {
    return &est[suffix_length][bit_num];
  }

 private:
  int index, suffix_length, bit_num;
  estimator est[8][8];
  estimator blank;
};

class CAVLCRemainingLevelEst : public CABACGenericEst {
  CodingType update(const int symbol, const int context) {
    return PIP_REMAINING_LEVEL;
  }
};
class CAVLCFirstLevelEst : public CABACGenericEst {
  CodingType update(const int symbol, const int context) {
    return PIP_FIRST_LEVEL;
  }
};

class CAVLCLevelSetupEst : public EstimatorContext {
 public:
  void begin(const int zz_index, const int param0, const int param1) {
    bit_num = 0;
    trailing_ones = param0;
  }
  CodingType update(const int symbol, const int context) {
    bit_num++;
    return PIP_LEVEL_SETUP;
  }
  estimator* get_estimator(const int context) {
    return &est[trailing_ones][bit_num];
  }

 private:
  int trailing_ones, bit_num;
  estimator est[8][3];
};

class CAVLCCoeffTokenChromaEst : public EstimatorContext {
 public:
  void begin(const int zz_index, const int param0, const int param1) {
    bit_num = 0;
  }
  CodingType update(const int symbol, const int context) {
    bit_num++;
    return PIP_COEFF_TOKEN_CHROMA;
  }
  estimator* get_estimator(const int context) {
    return &est[mb_coord.scan8_index][bit_num];
  }

 private:
  int bit_num;
  estimator est[64][8];
};

class CAVLCCoeffTokenEst : public EstimatorContext {
 public:
  void begin(const int zz_index, const int param0, const int param1) {
    total_coeff = param0;
    bit_num = 0;
  }
  CodingType update(const int symbol, const int context) {
    bit_num++;
    return PIP_COEFF_TOKEN;
  }
  estimator* get_estimator(const int context) {
    return &est[mb_coord.scan8_index][total_coeff][sub_mb_cat][bit_num];
  }

 private:
  int total_coeff, bit_num = 0;
  estimator est[64][17][5][12];
};