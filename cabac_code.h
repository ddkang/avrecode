//
// Arithmetic coding for H.264's CABAC encoding.
//

#pragma once

#include "arithmetic_code.h"

extern "C" {
#include "libavcodec/cabac.h"
static const uint8_t * const ff_h264_lps_range = ff_h264_cabac_tables + H264_LPS_RANGE_OFFSET;
static const uint8_t * const ff_h264_mlps_state = ff_h264_cabac_tables + H264_MLPS_STATE_OFFSET;
}


struct cavlc {
  template <typename OutputIterator>
  class encoder {
   public:
    explicit encoder(OutputIterator out) : out(out) {}

    size_t put(int symbol) {
      buffer = (buffer << 1) + (symbol & 1);
      index++;
      if (index == 8) {
        *out++ = buffer;
        buffer = 0;
        index = 0;
        return 1;
      }
      return 0;
    }

    size_t put_last() {
      if (index != 0) {
        // *out++ = buffer;
        while (put(0) != 1);
        return 1;
      } else
        return 0;
    }

   private:
    OutputIterator out;
    uint8_t buffer = 0;
    int index = 0;
  };
};

struct cabac {
  // Word size for encoder/decoder state. Reasonable values: uint64_t, uint32_t.
  typedef uint32_t fixed_point;
  // Word size for compressed data. Reasonable values: uint16_t, uint8_t.
  typedef uint16_t compressed_digit;
  // min_range must be at least 0x200 so that range/2 never rounds in put_bypass.
  static constexpr int min_range = 0x200;

  typedef arithmetic_code<fixed_point, compressed_digit, min_range> cabac_arithmetic_code;

  template <typename OutputIterator>
  class encoder {
   public:
    // Initial range is set so that (range >> normalize) == 0x1FE as required by CABAC spec.
    explicit encoder(OutputIterator out) : e(out, (cabac_arithmetic_code::fixed_one/0x200)*0x1FE) {}

    // Translate CABAC tables into generic arithmetic coding.
    size_t put(int symbol, uint8_t* state) {
      bool is_less_probable_symbol = (symbol != ((*state) & 1));
      size_t retval = e.put(is_less_probable_symbol, [state](fixed_point range) {
        // Find the normalizer such that range >> normalize is between 0x100 and 0x200.
        int normalize = log2(range / 0x100);
        // Use the most significant two bits of range (other than the leading 1) as an index into the table.
        int range_approx = int(range >> (normalize-1));
        fixed_point range_of_less_probable_symbol = ff_h264_lps_range[(range_approx & 0x180) + *state];
        return range_of_less_probable_symbol << normalize;
      });
      if (is_less_probable_symbol) {
        *state = ff_h264_mlps_state[127 - *state];
      } else {
        *state = ff_h264_mlps_state[128 + *state];
      }
      return retval;
    }

    // Simple implementation: put_bypass assumes a symbol probability of exactly 1/2.
    size_t put_bypass(int symbol) {
      return e.put(symbol, [](fixed_point range) { return range/2; });
    }

    // The end of stream symbol is always assumed to have probability ~2/256.
    size_t put_terminate(int end_of_stream_symbol) {
      size_t retval = e.put(end_of_stream_symbol, [](fixed_point range) {
        int normalize = log2(range / 0x100);
        return fixed_point(2) << normalize;
      });
      
      if (end_of_stream_symbol) {
        e.finish();
      }
      return retval;
    }

   private:
    static int log2(uint64_t x) {
      int i = 0;
      if (x >> 32) { x >>= 32; i += 32; }
      if (x >> 16) { x >>= 16; i += 16; }
      if (x >>  8) { x >>=  8; i +=  8; }
      if (x >>  4) { x >>=  4; i +=  4; }
      if (x >>  2) { x >>=  2; i +=  2; }
      if (x >>  1) { x >>=  1; i +=  1; }
      return i;
    }

    cabac_arithmetic_code::encoder<OutputIterator, uint8_t> e;
  };

  class decoder {
  };
};
