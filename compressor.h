extern "C" {
#include "libavcodec/avcodec.h"
#include "libavcodec/cabac.h"
#include "libavcodec/coding_hooks.h"
#include "libavformat/avformat.h"
#include "libavformat/avio.h"
#include "libavutil/error.h"
#include "libavutil/file.h"
}

#include <stdio.h>

#include "arithmetic_code.h"
#include "cabac_code.h"
#include "recode.pb.h"
#include "framebuffer.h"
#include "recode.h"
#include "av_decoder.h"
#include "h264_model.h"
#include "h264_symbol.h"

#pragma once

class compressor {
 public:
  compressor(const std::string &input_filename, std::ostream &out_stream)
      : input_filename(input_filename), out_stream(out_stream) {
    COUNT_TOTAL_SYMBOLS = 0;
    LOG_OUT = fopen("comp.log", "wb");
    if (av_file_map(input_filename.c_str(), &original_bytes, &original_size, 0, NULL) < 0) {
      throw std::invalid_argument("Failed to open file: " + input_filename);
    }
  }

  ~compressor() {
    av_file_unmap(original_bytes, original_size);
  }

  void run() {
    // Run through all the frames in the file, building the output using our hooks.
    av_decoder<compressor> d(this, input_filename);
    d.dump_stream_info();
    d.decode_video();

    // Flush the final block to the output and write to stdout.
    out.add_block()->set_literal(
        &original_bytes[prev_coded_block_end], original_size - prev_coded_block_end);
    out_stream << out.SerializeAsString();
    fclose(LOG_OUT);
    fprintf(stderr, "%d %d %d %d %d\n", TOTAL_NUM, NUM_2X2, NUM_4X4, NUM_WEIRD, NUM_8X8);
  }

  int read_packet(uint8_t *buffer_out, int size) {
    size = std::min(size, int(original_size - read_offset));
    memcpy(buffer_out, &original_bytes[read_offset], size);
    read_offset += size;
    return size;
  }

  class cabac_decoder {
   public:
    cabac_decoder(compressor *c, CABACContext *ctx_in, const uint8_t *buf,
                  uint8_t *state_start, int size) {
      out = c->find_next_coded_block_and_emit_literal(buf, size);
      model = nullptr;
      this->state_start = state_start;
      if (out == nullptr) {
        // We're skipping this block, so disable calls to our hooks.
        ctx_in->coding_hooks = nullptr;
        ctx_in->coding_hooks_opaque = nullptr;
        ::ff_reset_cabac_decoder(ctx_in, buf, size);
        return;
      }

      out->set_size(size);

      ctx = *ctx_in;
      ctx.coding_hooks = nullptr;
      ctx.coding_hooks_opaque = nullptr;
      ::ff_reset_cabac_decoder(&ctx, buf, size);

      this->c = c;
      model = &c->model;
      model->reset();
    }

    ~cabac_decoder() { assert(out == nullptr || out->has_cabac()); }

    void execute_symbol(int symbol, int state) {
      h264_symbol sym(symbol, state);
#define QUEUE_MODE
#ifdef QUEUE_MODE
      if (queueing_symbols == PIP_SIGNIFICANCE_MAP || queueing_symbols == PIP_SIGNIFICANCE_EOB ||
          !symbol_buffer.empty()) {
        symbol_buffer.push_back(sym);
        model->update_state_tracking(symbol);
      } else {
#endif
        sym.execute(encoder, model, out, encoder_out);
#ifdef QUEUE_MODE
      }
#endif
    }

    int get(uint8_t *state) {
      int symbol = ::ff_get_cabac(&ctx, state);
      execute_symbol(symbol, state - this->state_start);
      return symbol;
    }

    int get_bypass() {
      int symbol = ::ff_get_cabac_bypass(&ctx);
      execute_symbol(symbol, model->bypass_context);
      return symbol;
    }

    int get_terminate() {
      int n = ::ff_get_cabac_terminate(&ctx);
      int symbol = (n != 0);
      execute_symbol(symbol, model->terminate_context);
      return symbol;
    }

    void begin_coding_type(
        CodingType ct, int zigzag_index, int param0, int param1) {
      if (!model) {
        return;
      }
      bool begin_queue = model->begin_coding_type(ct, zigzag_index, param0, param1);
      if (begin_queue && (ct == PIP_SIGNIFICANCE_MAP || ct == PIP_SIGNIFICANCE_EOB)) {
        push_queueing_symbols(ct);
      }
    }

    void end_coding_type(CodingType ct) {
      if (!model) {
        return;
      }
      model->end_coding_type(ct);

      if ((ct == PIP_SIGNIFICANCE_MAP || ct == PIP_SIGNIFICANCE_EOB)) {
        stop_queueing_symbols();
        model->finished_queueing(ct,
                                 [&](h264_model::estimator *e, int *symbol) {
                                   size_t billable_bytes = encoder.put(*symbol, [&](range_t range) {
                                     return model->probability_for_model_key(range, e);
                                   });
                                   model->update_state_for_model_key(*symbol, e);
                                   if (billable_bytes) {
                                     model->billable_bytes(billable_bytes);
                                   }
                                 });
        static int i = 0;
        if (i++ < 10) {
          std::cerr << "FINISHED QUEUING DECODE: " <<
          (int) (model->frames[model->cur_frame].meta_at(model->mb_coord.mb_x,
                                                         model->mb_coord.mb_y).num_nonzeros[model->mb_coord.scan8_index]) <<
          std::endl;
        }
        pop_queueing_symbols(ct);
        model->coding_type = PIP_UNKNOWN;
      }
      // LOG_NEIGHBORS("\n");
    }

    void copy_coefficients(int16_t *block, int max_coeff) {
      if (!model) {
        return;
      }
      model->copy_coefficients(block, max_coeff);
    }

   private:
    void push_queueing_symbols(CodingType ct) {
      // Does not currently support nested queues.
      assert(queueing_symbols == PIP_UNKNOWN);
      assert(symbol_buffer.empty());
      queueing_symbols = ct;
    }

    void stop_queueing_symbols() {
      assert(queueing_symbols != PIP_UNKNOWN);
      queueing_symbols = PIP_UNKNOWN;
    }

    void pop_queueing_symbols(CodingType ct) {
      //std::cerr<< "FINISHED QUEUEING "<< symbol_buffer.size()<<std::endl;
      if (ct == PIP_SIGNIFICANCE_MAP || ct == PIP_SIGNIFICANCE_EOB) {
        if (model) {
          model->reset_mb_significance_state_tracking();
        }
      }
      for (auto &sym : symbol_buffer) {
        sym.execute(encoder, model, out, encoder_out);
      }
      symbol_buffer.clear();
    }

    Recoded::Block *out;
    CABACContext ctx;
    uint8_t *state_start;

    compressor *c;
    h264_model *model;
    std::vector <uint8_t> encoder_out;
    recoded_code::encoder <std::back_insert_iterator<std::vector < uint8_t>>, uint8_t>

    encoder {
      std::back_inserter(encoder_out)
    };

    CodingType queueing_symbols = PIP_UNKNOWN;
    std::vector <h264_symbol> symbol_buffer;
  };

  h264_model *get_model() {
    return &model;
  }

 private:

  Recoded::Block *find_next_coded_block_and_emit_literal(const uint8_t *buf, int size) {
    uint8_t *found = static_cast<uint8_t *>( memmem(
        &original_bytes[prev_coded_block_end], read_offset - prev_coded_block_end,
        buf, size));
    if (found && size >= SURROGATE_MARKER_BYTES) {
      size_t gap = found - &original_bytes[prev_coded_block_end];
      out.add_block()->set_literal(&original_bytes[prev_coded_block_end], gap);
      prev_coded_block_end += gap + size;
      Recoded::Block *newBlock = out.add_block();
      newBlock->set_length_parity(size & 1);
      if (size > 1) {
        newBlock->set_last_byte(&(buf[size - 1]), 1);
      }
      return newBlock;  // Return a block for the recoder to fill.
    } else {
      // Can't recode this block, probably because it was NAL-escaped. Place
      // a skip marker in the block list.
      Recoded::Block *block = out.add_block();
      block->set_skip_coded(true);
      block->set_size(size);
      return nullptr;  // Tell the recoder to ignore this block.
    }
  }

  std::string input_filename;
  std::ostream &out_stream;

  uint8_t *original_bytes = nullptr;
  size_t original_size = 0;
  int read_offset = 0;
  int prev_coded_block_end = 0;

  h264_model model;
  Recoded out;
};