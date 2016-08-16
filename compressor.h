extern "C" {
#include "libavcodec/avcodec.h"
#include "libavcodec/cabac.h"
#include "libavcodec/coding_hooks.h"
#include "libavcodec/get_bits.h"
#include "libavcodec/golomb.h"
#include "libavcodec/h264_cavlc.h"
#include "libavformat/avformat.h"
#include "libavformat/avio.h"
#include "libavutil/error.h"
#include "libavutil/file.h"
}

#include <bitset>
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
    LOG_OUT = fopen("logs/comp.log", "wb");
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

  class generic_decoder {
   public:
    void execute_symbol(int symbol, int state) {
      h264_symbol sym(symbol, state);
#define QUEUE_MODE
#ifdef QUEUE_MODE
      if (queueing_symbols == PIP_SIGNIFICANCE_MAP || queueing_symbols == PIP_SIGNIFICANCE_EOB ||
          !symbol_buffer.empty()) {
        symbol_buffer.push_back(sym);
        model->update_state_tracking(symbol, state);
      } else {
#endif
        sym.execute(encoder, model, out, encoder_out);
#ifdef QUEUE_MODE
      }
#endif
    }

    void begin_coding_type(CodingType ct, int zigzag_index, int param0, int param1) {
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
                                 [&](h264_model::estimator *e, int *symbol, int context) {
                                   size_t billable_bytes = encoder.put(*symbol, [&](range_t range) {
                                     return model->probability_for_model_key(range, e);
                                   });
                                   model->update_state_for_model_key(*symbol, context, e);
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
      if (!model) return;
      model->copy_coefficients(block, max_coeff);
    }

    void set_mb_type(int mb_type) {
      if (!model) return;
      model->set_mb_type(mb_type);
    }

   protected:
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
    uint8_t *state_start;

    compressor *c;
    h264_model *model;
    std::vector <uint8_t> encoder_out;
    recoded_code::encoder<std::back_insert_iterator<std::vector<uint8_t> >, uint8_t> encoder {
      std::back_inserter(encoder_out)
    };

    CodingType queueing_symbols = PIP_UNKNOWN;
    std::vector <h264_symbol> symbol_buffer;
  };

  class cavlc_decoder : public generic_decoder {
   public:
    cavlc_decoder(compressor *c, GetBitContext *ctx_in, const uint8_t *buf, int size) {
      out = c->find_next_coded_block_and_emit_literal(buf, size);
      model = nullptr;
      if (out == nullptr) {
        // We're skipping this block, so disable calls to our hooks.
        ctx_in->cavlc_hooks = nullptr;
        ctx_in->cavlc_hooks_opaque = nullptr;
        return;
      }

      out->set_size(size);
      ff_ctx = ctx_in;
      gb_ctx = *ctx_in;
      gb_ctx.cavlc_hooks = nullptr;
      gb_ctx.cavlc_hooks_opaque = nullptr;

      this->c = c;
      model = &c->model;
      model->reset();

      const std::bitset<32> gb_size_bits(ctx_in->size_in_bits);
      const std::bitset<32> gb_index(ctx_in->index);
      for (int i = 0; i < gb_size_bits.size(); i++)
        execute_symbol(gb_size_bits[i], 0);
      for (int i = 0; i < gb_index.size(); i++)
        execute_symbol(gb_index[i], 0);
    }

    void execute_golomb(unsigned symbol, bool print) {
      const int len = av_log2(symbol + 1) * 2 + 1;
      const std::bitset<32> bits(symbol + 1); // The last len bits
      for (int i = len - 1; i >= 0; i--)
        execute_symbol(bits[i], 0);
      {
        static int FIRST_TIME_COMP = 0;
        if (print && FIRST_TIME_COMP++ < 10)
          fprintf(stderr, "%s %d %d %d\n", bits.to_string().c_str(), symbol, len, gb_ctx.index);
      }
    }
    void execute_golomb(unsigned symbol) {
      execute_golomb(symbol, true);
    }

    int get_ue_golomb() {
      const int prev_index = gb_ctx.index;
      int symbol = ::get_ue_golomb(&gb_ctx);
      const int index_diff = gb_ctx.index - prev_index;
      static int UE_GOLOMB_COMP = 0;
      if (UE_GOLOMB_COMP++ <= 10) {
        gb_ctx.index -= index_diff;
        const int stream = ::show_bits(&gb_ctx, index_diff);
        const std::bitset<32> bits(stream);
        /*fprintf(stderr, "UE Golomb: %d %d %d %s\n",
                UE_GOLOMB_COMP, index_diff, symbol, bits.to_string().c_str());*/
        gb_ctx.index += index_diff;
      }
      execute_golomb(symbol);
      return symbol;
    }

    int get_ue_golomb_31() {
      int symbol = ::get_ue_golomb_31(&gb_ctx);
      execute_golomb(symbol);
      return symbol;
    }

    // Hypothetically, this could do bad things, but in practice it doesn't (I hope)
    unsigned get_ue_golomb_long() {
      // unsigned symbol = ::get_ue_golomb_long(&gb_ctx);
      unsigned symbol = ::get_ue_golomb(&gb_ctx);
      execute_golomb(symbol);
      return symbol;
    }

    int get_se_golomb() {
      const int prev_index = gb_ctx.index;
      const int symbol = ::get_se_golomb(&gb_ctx);
      const unsigned golomb_symbol = (symbol <= 0) ? -2 * symbol : 2 * symbol - 1;
      const int len = av_log2(golomb_symbol + 1) * 2 + 1;
      execute_golomb(golomb_symbol);
      static int SE_GOLOMB_COMP = 0;
      if (SE_GOLOMB_COMP++ <= 10)
        fprintf(stderr, "se g: %d %d %d %d %d\n",
                SE_GOLOMB_COMP, prev_index - gb_ctx.index, len, golomb_symbol, symbol);
      return symbol;
    }

    unsigned int get_bits(int n) {
      unsigned int symbol = 0;
      for (int i = 0; i < n; i++)
        symbol = (symbol << 1) + get_bits1();
      return symbol;
    }

    unsigned int get_bits1() {
      unsigned int symbol = ::get_bits1(&gb_ctx);
      execute_symbol(symbol, 0);
      return symbol;
    }

    int get_vlc2(int16_t (*table)[2], int bits, int max_depth) {
      const int prev_index = gb_ctx.index;
      const int symbol = ::get_vlc2(&gb_ctx, table, bits, max_depth);
      const int index_diff = gb_ctx.index - prev_index;

      gb_ctx.index = prev_index;
      const int write_symbol = ::get_bits(&gb_ctx, index_diff);
      for (int i = 0; i < index_diff; i++)
        execute_symbol( (write_symbol >> (index_diff - i - 1)) & 1, 0);

      {
        static int VLC_COMP = 0;
        if (VLC_COMP++ <= 10)
          fprintf(stderr, "vlc: %d %d %d %d %d %d\n", VLC_COMP, symbol, gb_ctx.index, max_depth, bits, index_diff);
      }

      return symbol;
    }

    int get_level_prefix() {
      int symbol = ::get_level_prefix(&gb_ctx);
      {
        static int LEVEL_PREFIX_COMP = 0;
        if (LEVEL_PREFIX_COMP++ <= 100)
          fprintf(stderr, "lp: %d %d %d\n", LEVEL_PREFIX_COMP, symbol, gb_ctx.index);
      }
      // The level prefix is encoded as unary
      for (int i = 0; i < symbol; i++)
        execute_symbol(0, 0);
      execute_symbol(1, 0);
      return symbol;
    }

    int last_show_bits = -1;
    int last_show_n = -1;
    unsigned int show_bits(int n) {
      const unsigned int symbol = ::show_bits(&gb_ctx, n);
      last_show_bits = symbol;
      last_show_n = n;
      {
        static int SHOW_BITS_COMP = 0;
        if (SHOW_BITS_COMP++ <= 10)
          fprintf(stderr, "sb: %d %d %d %d\n", SHOW_BITS_COMP, n, symbol, gb_ctx.index);
      }
      return symbol;
    }
    // Does not matter from the encode side
    unsigned int recode_show_bits(const int8_t table[256][2], int n) {
      return show_bits(n);
    }
    void skip_bits(int n) {
      ::skip_bits(&gb_ctx, n);
      const unsigned int symbol = last_show_bits;
      for (int i = 0; i < n; i++)
        execute_symbol( (symbol >> (last_show_n - i - 1)) & 1, 0 );
      last_show_bits = -1;
      last_show_n = -1;
    }

    void terminate() {
      encoder.finish();
      out->set_cabac(&encoder_out[0], encoder_out.size());

      // The bitstream is used elsewhere so set the hooks to null
      // Our local context has the hooks to null, so this does what we want
      *ff_ctx = gb_ctx;

      static int TERMINATE_COMP = 0;
      if (TERMINATE_COMP++ <= 300 && TERMINATE_COMP >= 100)
        fprintf(stderr, "gb ctx: %d %d %d %d\n", TERMINATE_COMP, gb_ctx.index, gb_ctx.size_in_bits, gb_ctx.size_in_bits_plus8);
    }

    ~cavlc_decoder() { assert(out == nullptr || out->has_cabac()); }

   private:
    GetBitContext gb_ctx;
    GetBitContext *ff_ctx;
  };

  class cabac_decoder : public generic_decoder {
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

    // It may be possible to predict the sign better than random bypass bits.
    int get_sign_bypass() {
      int symbol = ::ff_get_cabac_bypass(&ctx);
      execute_symbol(symbol, model->sign_bypass_context);
      return symbol;
    }

    int get_terminate() {
      int n = ::ff_get_cabac_terminate(&ctx);
      int symbol = (n != 0);
      execute_symbol(symbol, model->terminate_context);
      return symbol;
    }

   private:
    CABACContext ctx;
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
        newBlock->set_first_byte(buf, 1);
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