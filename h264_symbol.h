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
#include "recode.h"

#pragma once

class h264_symbol {
 public:
  h264_symbol(int symbol, const int state)
      : symbol(symbol), state(state) {
  }

  template<class T>
  void execute(T &encoder, h264_model *model,
               Recoded::Block *out, std::vector <uint8_t> &encoder_out) {
    if (model->coding_type != PIP_SIGNIFICANCE_EOB) {
      size_t billable_bytes = encoder.put(symbol, [&](range_t range) {
        return model->probability_for_state(range, state);
      });
      if (billable_bytes) {
        model->billable_bytes(billable_bytes);
      }
    }
    model->update_state(symbol, state);
    if (state == EstimatorContext::terminate_context && symbol) {
      encoder.finish();
      out->set_cabac(&encoder_out[0], encoder_out.size());
    }
  }

 private:
  int symbol;
  const int state;
};
