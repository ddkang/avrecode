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


bool get_neighbor_sub_mb(bool above, int sub_mb_size,
                         CoefficientCoord input,
                         CoefficientCoord *output) {
  int mb_x = input.mb_x;
  int mb_y = input.mb_y;
  int scan8_index = input.scan8_index;
  output->scan8_index = scan8_index;
  output->mb_x = mb_x;
  output->mb_y = mb_y;
  output->zigzag_index = input.zigzag_index;
  if (scan8_index >= 16 * 3) {
    if (above) {
      if (mb_y > 0) {
        output->mb_y -= 1;
        return true;
      }
      return false;
    } else {
      if (mb_x > 0) {
        output->mb_x -= 1;
        return true;
      }
      return false;
    }
  }
  int scan8 = scan_8[scan8_index];
  int left_shift = (above ? 0 : -1);
  int above_shift = (above ? -1 : 0);
  auto neighbor = reverse_scan_8[(scan8 >> 3) + above_shift][(scan8 & 7) + left_shift];
  if (neighbor.neighbor_left) {
    if (mb_x == 0) {
      return false;
    } else {
      --mb_x;
    }
  }
  if (neighbor.neighbor_up) {
    if (mb_y == 0) {
      return false;
    } else {
      --mb_y;
    }
  }
  output->scan8_index = neighbor.scan8_index;
  if (sub_mb_size >= 32) {
    output->scan8_index /= 4;
    output->scan8_index *= 4; // round down to the nearest multiple of 4
  }
  output->zigzag_index = input.zigzag_index;
  output->mb_x = mb_x;
  output->mb_y = mb_y;
  return true;
}


bool get_neighbor(bool above, int sub_mb_size,
                  CoefficientCoord input,
                  CoefficientCoord *output) {
  int mb_x = input.mb_x;
  int mb_y = input.mb_y;
  int scan8_index = input.scan8_index;
  int zigzag_index = input.zigzag_index;
  int dimension = 2;
  if (sub_mb_size > 15) {
    dimension = 4;
  }
  if (sub_mb_size > 32) {
    dimension = 8;
  }
  if (scan8_index >= 16 * 3) {
    // we are DC...
    int linear_index = unzigzag4[zigzag_index];
    if (sub_mb_size == 16) {
      linear_index = unzigzag16[zigzag_index];
    } else {
      assert(sub_mb_size <= 4);
    }
    if ((above && linear_index >= dimension) // if is inner
        || ((linear_index & (dimension - 1)) && !above)) {
      if (above) {
        linear_index -= dimension;
      } else {
        --linear_index;
      }
      if (sub_mb_size == 16) {
        output->zigzag_index = zigzag16[linear_index];
      } else {
        output->zigzag_index = zigzag4[linear_index];
      }
      output->mb_x = mb_x;
      output->mb_y = mb_y;
      output->scan8_index = scan8_index;
      return true;
    }
    if (above) {
      if (mb_y == 0) {
        return false;
      }
      linear_index += dimension * (dimension - 1);//go to bottom
      --mb_y;
    } else {
      if (mb_x == 0) {
        return false;
      }
      linear_index += dimension - 1;//go to end of row
      --mb_x;
    }
    if (sub_mb_size == 16) {
      output->zigzag_index = zigzag16[linear_index];
    } else {
      output->zigzag_index = linear_index;
    }
    output->mb_x = mb_x;
    output->mb_y = mb_y;
    output->scan8_index = scan8_index;
    return true;
  }
  int scan8 = scan_8[scan8_index];
  int left_shift = (above ? 0 : -1);
  int above_shift = (above ? -1 : 0);
  auto neighbor = reverse_scan_8[(scan8 >> 3) + above_shift][(scan8 & 7) + left_shift];
  if (neighbor.neighbor_left) {
    if (mb_x == 0) {
      return false;
    } else {
      --mb_x;
    }
  }
  if (neighbor.neighbor_up) {
    if (mb_y == 0) {
      return false;
    } else {
      --mb_y;
    }
  }
  output->scan8_index = neighbor.scan8_index;
  if (sub_mb_size >= 32) {
    output->scan8_index /= 4;
    output->scan8_index *= 4; // round down to the nearest multiple of 4
  }
  output->zigzag_index = zigzag_index;
  output->mb_x = mb_x;
  output->mb_y = mb_y;
  return true;
}

bool get_neighbor_coefficient(bool above,
                              int sub_mb_size,
                              CoefficientCoord input,
                              CoefficientCoord *output) {
  if (input.scan8_index >= 16 * 3) {
    return get_neighbor(above, sub_mb_size, input, output);
  }
  int zigzag_addition = 0;

  if ((sub_mb_size & (sub_mb_size - 1)) != 0) {
    zigzag_addition = 1;// the DC is not included
  }
  const uint8_t *zigzag_to_raster = unzigzag16;
  const uint8_t *raster_to_zigzag = zigzag16;
  int dim = 4;
  if (sub_mb_size <= 4) {
    dim = 2;
    zigzag_to_raster = zigzag4;
    raster_to_zigzag = unzigzag4;
  }
  if (sub_mb_size > 16) {
    dim = 16;
    zigzag_to_raster = zigzag64;
    raster_to_zigzag = unzigzag64;
  }
  int raster_coord = zigzag_to_raster[input.zigzag_index + zigzag_addition];
  //fprintf(stderr, "%d %d   %d -> %d\n", sub_mb_size, zigzag_addition, input.zigzag_index, raster_coord);
  if (above) {
    if (raster_coord >= dim) {
      raster_coord -= dim;
    } else {
      return false;
    }
  } else {
    if (raster_coord & (dim - 1)) {
      raster_coord -= 1;
    } else {
      return false;
    }
  }
  *output = input;
  output->zigzag_index = raster_to_zigzag[raster_coord] - zigzag_addition;
  return true;
}


#define STRINGIFY_COMMA(s) #s ,
const char * billing_names [] = {EACH_PIP_CODING_TYPE(STRINGIFY_COMMA)};
#undef STRINGIFY_COMMA

class h264_model {
 public:
  CodingType coding_type = PIP_UNKNOWN;
  size_t bill[sizeof(billing_names) / sizeof(billing_names[0])];
  size_t cabac_bill[sizeof(billing_names) / sizeof(billing_names[0])];
  FrameBuffer frames[2];
  int cur_frame = 0;
  uint8_t STATE_FOR_NUM_NONZERO_BIT[6];
  bool do_print;
 public:
  h264_model() {
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

  ~h264_model() {
    bool first = true;
    for (size_t i = 0; i < sizeof(billing_names) / sizeof(billing_names[i]); ++i) {
      if (bill[i]) {
        if (first) {
          fprintf(stderr, "Avrecode Bill\n=============\n");
        }
        first = false;
        fprintf(stderr, "%s : %ld\n", billing_names[i], bill[i]);
      }
    }
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
    memset(STATE_FOR_NUM_NONZERO_BIT, 0, sizeof(STATE_FOR_NUM_NONZERO_BIT));
  }

  bool fetch(bool previous, bool match_type, CoefficientCoord coord, int16_t *output) const {
    if (match_type && (previous || coord.mb_x != mb_coord.mb_x || coord.mb_y != mb_coord.mb_y)) {
      BlockMeta meta = frames[previous ? !cur_frame : cur_frame].meta_at(coord.mb_x, coord.mb_y);
      if (!meta.coded) { // when we populate mb_type in the metadata, then we can use it here
        return false;
      }
    }
    *output = frames[previous ? !cur_frame : cur_frame].at(coord.mb_x, coord.mb_y).residual[coord.scan8_index * 16 +
                                                                                            coord.zigzag_index];
    return true;
  }

  model_key get_model_key(const uint8_t *context) const {
    switch (coding_type) {
      case PIP_SIGNIFICANCE_NZ:
        return model_key(context, 0, 0);
      case PIP_UNKNOWN:
      case PIP_UNREACHABLE:
      case PIP_RESIDUALS:
        return model_key(context, 0, 0);
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
        int cat_lookup[14] = {105 + 0, 105 + 15, 105 + 29, 105 + 44, 105 + 47, 402, 484 + 0, 484 + 15, 484 + 29, 660,
                              528 + 0, 528 + 15, 528 + 29, 718};
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
        assert(sub_mb_cat < (int) (sizeof(cat_lookup) / sizeof(cat_lookup[0])));
        int neighbor_above = 2;
        int neighbor_left = 2;
        int coeff_neighbor_above = 2;
        int coeff_neighbor_left = 2;
        if (do_print) {
          LOG_NEIGHBORS("[");
        }
        {
          CoefficientCoord neighbor_left_coord = {0, 0, 0, 0};
          if (get_neighbor(false, sub_mb_size, mb_coord, &neighbor_left_coord)) {
            int16_t tmp = 0;
            if (fetch(false, true, neighbor_left_coord, &tmp)) {
              neighbor_left = !!tmp;
              if (do_print) {
                LOG_NEIGHBORS("%d,", tmp);
              }
            } else {
              neighbor_left = 3;
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
          CoefficientCoord neighbor_above_coord = {0, 0, 0, 0};
          if (get_neighbor(true, sub_mb_size, mb_coord, &neighbor_above_coord)) {
            int16_t tmp = 0;
            if (fetch(false, true, neighbor_above_coord, &tmp)) {
              neighbor_above = !!tmp;
              if (do_print) {
                LOG_NEIGHBORS("%d,", tmp);
              }
            } else {
              neighbor_above = 3;
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
              coeff_neighbor_left = !!tmp;
            } else {
              coeff_neighbor_left = 3;
            }
          } else {
          }
        }
        {
          CoefficientCoord neighbor_above_coord = {0, 0, 0, 0};
          if (get_neighbor_coefficient(true, sub_mb_size, mb_coord, &neighbor_above_coord)) {
            int16_t tmp = 0;
            if (fetch(false, true, neighbor_above_coord, &tmp)) {
              coeff_neighbor_above = !!tmp;
            } else {
              coeff_neighbor_above = 3;
            }
          } else {
          }
        }

        // FIXM: why doesn't this prior help at all
        {
          int16_t output = 0;
          if (fetch(true, true, mb_coord, &output)) {
            if (do_print) LOG_NEIGHBORS("%d] ", output);
          } else {
            if (do_print) LOG_NEIGHBORS("x] ");
          }
        }
        //const BlockMeta &meta = frames[!cur_frame].meta_at(mb_x, mb_y);
        int num_nonzeros = frames[cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y).num_nonzeros[mb_coord.scan8_index];
        (void) neighbor_above;
        (void) neighbor_left;
        (void) coeff_neighbor_above;
        (void) coeff_neighbor_left;//haven't found a good way to utilize these priors to make the results better
        return model_key(&significance_context,
                         64 * num_nonzeros + nonzeros_observed,
                         sub_mb_is_dc + zigzag_offset * 2 + 16 * 2 * cat_lookup[sub_mb_cat]);
      }
      case PIP_SIGNIFICANCE_EOB: {
        // FIXME: why doesn't this prior help at all
        int num_nonzeros = frames[cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y).num_nonzeros[mb_coord.scan8_index];

        return model_key(NULL, num_nonzeros == nonzeros_observed, 0);
      }
      default:
        break;
    }
    assert(false && "Unreachable");
    abort();
  }

  range_t probability_for_model_key(range_t range, model_key key) {
    auto *e = &estimators[key];
    int total = e->pos + e->neg;
    return (range / total) * e->pos;
  }

  range_t probability_for_state(range_t range, const uint8_t *context) {
    return probability_for_model_key(range, get_model_key(context));
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
      bool block_of_interest = (sub_mb_cat == 1 || sub_mb_cat == 2);
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
        bool has_left = get_neighbor_sub_mb(false, sub_mb_size, mb_coord, &neighbor);
        if (has_left) {
          left_nonzero = frames[cur_frame].meta_at(neighbor.mb_x, neighbor.mb_y).num_nonzeros[neighbor.scan8_index];
        }
        bool has_above = get_neighbor_sub_mb(true, sub_mb_size, mb_coord, &neighbor);
        if (has_above) {
          above_nonzero = frames[cur_frame].meta_at(neighbor.mb_x, neighbor.mb_y).num_nonzeros[neighbor.scan8_index];
        }

        do {
          uint32_t cur_bit = (1 << i);
          int left_nonzero_bit = 2;
          if (has_left) {
            left_nonzero_bit = (left_nonzero >= cur_bit);
          }
          int above_nonzero_bit = 2;
          if (above_nonzero) {
            above_nonzero_bit = (above_nonzero >= cur_bit);
          }
          put_or_get(
              model_key(&(STATE_FOR_NUM_NONZERO_BIT[i]),
                        serialized_so_far + 64 * (frames[!cur_frame].meta_at(
                            mb_coord.mb_x, mb_coord.mb_y).num_nonzeros[mb_coord.scan8_index] >= cur_bit) +
                        128 * left_nonzero_bit +
                        384 * above_nonzero_bit,
                        meta.is_8x8 + sub_mb_is_dc * 2 + sub_mb_chroma422 + sub_mb_cat * 4), &nonzero_bits[i]);
          if (nonzero_bits[i]) {
            serialized_so_far |= cur_bit;
          }
        } while (++i < serialized_bits);

        if (block_of_interest) {
          LOG_NEIGHBORS("%d ", mb_coord.scan8_index);
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
        LOG_NEIGHBORS("} %d> ", meta.num_nonzeros[mb_coord.scan8_index]);
      }
      coding_type = last;
    }
  }

  void end_coding_type(CodingType ct) {
    if (ct == PIP_SIGNIFICANCE_MAP) {
      assert(coding_type == PIP_UNREACHABLE
             || (coding_type == PIP_SIGNIFICANCE_MAP && mb_coord.zigzag_index == 0));
      uint8_t num_nonzeros = 0;
      for (int i = 0; i < sub_mb_size; ++i) {
        int16_t res = frames[cur_frame].at(mb_coord.mb_x, mb_coord.mb_y).residual[mb_coord.scan8_index * 16 + i];
        assert(res == 1 || res == 0);
        if (res != 0) {
          num_nonzeros += 1;
        }
      }
      BlockMeta &meta = frames[cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y);
      meta.is_8x8 = meta.is_8x8 || (sub_mb_size > 32); // 8x8 will have DC be 2x2
      meta.coded = true;
      assert(meta.num_nonzeros[mb_coord.scan8_index] == 0 || meta.num_nonzeros[mb_coord.scan8_index] == num_nonzeros);
      meta.num_nonzeros[mb_coord.scan8_index] = num_nonzeros;
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
        if (sub_mb_is_dc) {
          mb_coord.zigzag_index = 0;
        } else {
          mb_coord.zigzag_index = 0;
        }
        begin_queueing = true;
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

  void update_state_tracking(int symbol) {
    switch (coding_type) {
      case PIP_SIGNIFICANCE_NZ:
        break;
      case PIP_SIGNIFICANCE_MAP:
        frames[cur_frame].at(mb_coord.mb_x, mb_coord.mb_y).residual[mb_coord.scan8_index * 16 +
                                                                    mb_coord.zigzag_index] = symbol;
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
      case PIP_UNKNOWN:
        break;
      case PIP_UNREACHABLE:
        assert(false);
      default:
        assert(false);
    }
  }

  void update_state(int symbol, const uint8_t *context) {
    update_state_for_model_key(symbol, get_model_key(context));
  }

  void update_state_for_model_key(int symbol, model_key key) {
    if (coding_type == PIP_SIGNIFICANCE_EOB) {
      int num_nonzeros = frames[cur_frame].meta_at(mb_coord.mb_x, mb_coord.mb_y).num_nonzeros[mb_coord.scan8_index];
      assert(symbol == (num_nonzeros == nonzeros_observed));
    }
    auto *e = &estimators[key];
    if (symbol) {
      e->pos++;
    } else {
      e->neg++;
    }
    if ((coding_type != PIP_SIGNIFICANCE_MAP && e->pos + e->neg > 0x60)
        || (coding_type == PIP_SIGNIFICANCE_MAP && e->pos + e->neg > 0x50)) {
      e->pos = (e->pos + 1) / 2;
      e->neg = (e->neg + 1) / 2;
    }
    update_state_tracking(symbol);
  }

  const uint8_t bypass_context = 0, terminate_context = 0, significance_context = 0;
  CoefficientCoord mb_coord;
  int nonzeros_observed = 0;
  int sub_mb_cat = -1;
  int sub_mb_size = -1;
  int sub_mb_is_dc = 0;
  int sub_mb_chroma422 = 0;
 private:
  struct estimator {
    int pos = 1, neg = 1;
  };
  std::map <model_key, estimator> estimators;
};