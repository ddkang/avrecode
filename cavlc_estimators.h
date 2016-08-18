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
  GolombEstimator est[16][16][2];
  int index;
  bool is_x;
};