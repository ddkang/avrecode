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

class decompressor {
  // Used to track the decoding state of each block.
  struct block_state {
    bool coded = false;
    std::string surrogate_marker;
    std::string out_bytes;
    bool done = false;
    int8_t length_parity = -1;
    uint8_t last_byte;
  };

 public:
  decompressor(const std::string &input_filename, std::ostream &out_stream)
      : input_filename(input_filename), out_stream(out_stream) {
    COUNT_TOTAL_SYMBOLS = 0;
    LOG_OUT = fopen("logs/decomp.log", "wb");
    uint8_t *bytes;
    size_t size;
    if (av_file_map(input_filename.c_str(), &bytes, &size, 0, NULL) < 0) {
      throw std::invalid_argument("Failed to open file: " + input_filename);
    }
    in.ParseFromArray(bytes, size);
  }

  decompressor(const std::string &input_filename, const std::string &in_bytes, std::ostream &out_stream)
      : input_filename(input_filename), out_stream(out_stream) {
    COUNT_TOTAL_SYMBOLS = 0;
    LOG_OUT = fopen("logs/decomp.log", "wb");
    in.ParseFromString(in_bytes);
  }

  void run() {
    blocks.clear();
    blocks.resize(in.block_size());

    av_decoder<decompressor> d(this, input_filename);
    d.decode_video();

    for (auto &block : blocks) {
      if (!block.done) throw std::runtime_error("Not all blocks were decoded.");
      if (block.length_parity != -1) {
        // Correct for x264 padding: replace last byte or add an extra byte.
        if (block.length_parity != (int) (block.out_bytes.size() & 1)) {
          block.out_bytes.insert(block.out_bytes.end(), block.last_byte);
        } else {
          block.out_bytes[block.out_bytes.size() - 1] = block.last_byte;
        }
      }
      out_stream << block.out_bytes;
    }
    fclose(LOG_OUT);
  }

  int read_packet(uint8_t *buffer_out, int size) {
    uint8_t *p = buffer_out;
    while (size > 0 && read_index < in.block_size()) {
      if (read_block.empty()) {
        const Recoded::Block &block = in.block(read_index);
        if (int(block.has_literal()) + int(block.has_cabac()) + int(block.has_skip_coded()) != 1) {
          throw std::runtime_error("Invalid input block: must have exactly one type");
        }
        if (block.has_literal()) {
          // This block is passed through without any re-coding.
          blocks[read_index].out_bytes = block.literal();
          blocks[read_index].done = true;
          read_block = block.literal();
        } else if (block.has_cabac()) {
          // Re-coded CABAC coded block. out_bytes will be filled by cabac_decoder.
          blocks[read_index].coded = true;
          blocks[read_index].surrogate_marker = next_surrogate_marker();
          blocks[read_index].done = false;
          if (!block.has_size()) {
            throw std::runtime_error("CABAC block requires size field.");
          }
          if (block.has_length_parity() && block.has_last_byte() &&
              !block.last_byte().empty()) {
            blocks[read_index].length_parity = block.length_parity();
            blocks[read_index].last_byte = block.last_byte()[0];
          }
          read_block = make_surrogate_block(blocks[read_index].surrogate_marker, block.size());
          // read_block.resize(block.size(), 0);
          // The first x bytes are used by the bitstream reader
          memcpy((uint8_t *) read_block.c_str(), (uint8_t *) block.cabac().data(),
                 std::min(1 << 30, (int) block.size()));
        } else if (block.has_skip_coded() && block.skip_coded()) {
          // Non-re-coded CABAC coded block. The bytes of this block are
          // emitted in a literal block following this one. This block is
          // a flag to expect a cabac_decoder without a surrogate marker.
          blocks[read_index].coded = true;
          blocks[read_index].done = true;
        } else {
          throw std::runtime_error("Unknown input block type");
        }
      }
      if ((size_t) read_offset < read_block.size()) {
        int n = read_block.copy(reinterpret_cast<char *>(p), size, read_offset);
        read_offset += n;
        p += n;
        size -= n;
      }
      if ((size_t) read_offset >= read_block.size()) {
        read_block.clear();
        read_offset = 0;
        read_index++;
      }
    }
    return p - buffer_out;
  }

  class generic_decoder {
   public:
    void begin_coding_type(
        CodingType ct, int zigzag_index, int param0, int param1) {
      bool begin_queue = model && model->begin_coding_type(ct, zigzag_index, param0, param1);
      if (begin_queue && ct) {
        model->finished_queueing(ct,
                                 [&](h264_model::estimator *e, int *symbol, int context) {
                                   *symbol = decoder->get([&](range_t range) {
                                     return model->probability_for_model_key(range, e);
                                   });
                                   model->update_state_for_model_key(*symbol, context, e);
                                 });
        static int i = 0;
        if (i++ < 10) {
          std::cerr << "FINISHED QUEUING RECODE: " <<
                    (int) model->frames[model->cur_frame].meta_at(model->mb_coord.mb_x,
                                                                  model->mb_coord.mb_y).num_nonzeros[model->mb_coord.scan8_index] <<
                    std::endl;
        }
      }
    }

    void end_coding_type(CodingType ct) {
      if (!model) {
        return;
      }
      model->end_coding_type(ct);
    }

    void copy_coefficients(int16_t *block, int max_coeff) {
      if (!model) return;
      model->copy_coefficients(block, max_coeff);
    }

    void set_mb_type(int mb_type) {
      if (!model) return;
      model->set_mb_type(mb_type);
    }

    virtual void finish() = 0;

   protected:
    int index;
    const Recoded::Block *block;
    block_state *out = nullptr;

    h264_model *model;
    std::unique_ptr<recoded_code::decoder<const char *, uint8_t>> decoder;

    std::vector <uint8_t> data_out;
    cabac::encoder<std::back_insert_iterator < std::vector < uint8_t>>> cabac_encoder {
      std::back_inserter(data_out)
    };

    cavlc::encoder<std::back_insert_iterator<std::vector <uint8_t> > > cavlc_encoder {
      std::back_inserter(data_out)
    };
  };

  class cavlc_decoder : public generic_decoder {
   public:
    cavlc_decoder(decompressor *d, GetBitContext *ctx_in, const uint8_t *buf, int size) {
      index = d->recognize_coded_block(buf, size);
      block = &d->in.block(index);
      out = &d->blocks[index];
      model = nullptr;

      if (block->has_cabac()) {
        // Cheat mode
        ctx_in->size_in_bits = block->cabac().size() * 8;
        ctx_in->size_in_bits_plus8 = ctx_in->size_in_bits + 8;
        ctx_in->buffer_end = ctx_in->buffer + block->cabac().size();

        model = &d->model;
        model->reset();
        decoder.reset(new recoded_code::decoder<const char *, uint8_t>(
            block->cabac().data(), block->cabac().data() + block->cabac().size()));

        //data_out.resize(block->size());
        //memcpy(&data_out[0], (uint8_t *) block->cabac().data(), block->size());
        /*ctx_in->cavlc_hooks = nullptr;
        ctx_in->cavlc_hooks_opaque = nullptr;
        finish();*/
      } else if (block->has_skip_coded() && block->skip_coded()) {
        // We're skipping this block, so disable calls to our hooks.
        ctx_in->cavlc_hooks = nullptr;
        ctx_in->cavlc_hooks_opaque = nullptr;
      } else {
        throw std::runtime_error("Expected CAVLC block.");
      }
      cabac_state.resize(1024);
      ff_ctx = ctx_in;
    }
    std::vector<uint8_t> cabac_state;
    GetBitContext *ff_ctx;

    ~cavlc_decoder() { assert(out->done); }

    int get() {
      const int state = 0;
      int symbol = decoder->get([&](range_t range) {
        return model->probability_for_state(range, state);
      });
      size_t billable_bytes = cavlc_encoder.put(symbol);
      if (billable_bytes) {
        model->billable_cabac_bytes(billable_bytes);
      }
      model->update_state(symbol, state);
      return symbol;
    }

    unsigned decode_golomb(bool print) {
      std::bitset<32> bits(0);
      int zeros = 0;
      while (0 == get() && zeros < 32) zeros++;

      bits[zeros] = 1;
      for (int i = 0; i < zeros; i++)
        bits[zeros - i - 1] = get();
      int symbol = bits.to_ulong() - 1;

      const int len = zeros * 2 + 1;
      ff_ctx->index += len;
      {
        static int FIRST_TIME_DECOMP = 0;
        if (print && FIRST_TIME_DECOMP++ < 10)
          fprintf(stderr, "%s %d %d %d\n", bits.to_string().c_str(), symbol, len, ff_ctx->index);
      }
      return symbol;
    }
    unsigned decode_golomb() {
      return decode_golomb(true);
    }

    int get_ue_golomb() {
      return decode_golomb();
    }
    int get_ue_golomb_31() {
      return decode_golomb();
    }
    unsigned get_ue_golomb_long() {
      return decode_golomb();
    }
    int get_se_golomb() {
      const int base_symbol = decode_golomb();
      const int symbol = base_symbol % 2 == 0 ? -base_symbol / 2 : base_symbol / 2 + 1;
      static int SE_GOLOMB_DECOMP = 0;
      if (SE_GOLOMB_DECOMP++ <= 10) fprintf(stderr, "se g: %d %d %d\n", SE_GOLOMB_DECOMP, base_symbol, symbol);
      return symbol;
      /*if (base_symbol % 2 == 0)
        return -base_symbol / 2;
      else
        return base_symbol / 2 + 1; // +1??*/
    }
    unsigned int get_bits(int n) {
      int symbol = 0;
      for (int i = 0; i < n; i++)
        symbol |= get() << (n - i - 1);

      ff_ctx->index += n;

      return symbol;
    }
    unsigned int get_bits1() {
      ff_ctx->index += 1;
      return get();
    }
    int get_vlc2(int16_t (*table)[2], int bits, int max_depth) {
      int symbol = decode_golomb(false);

      {
        const int len = av_log2(symbol + 1) * 2 + 1;
        ff_ctx->index -= len;
      }
      {
        const bool read_twice = (max_depth == 2) ? get() : false;
        int table_ind = 0;
        while (table[table_ind][0] != symbol) table_ind++;
        ff_ctx->index += table[table_ind][1];
        if (read_twice)
          ff_ctx->index += bits;
      }
      {
        static int VLC_DECOMP = 0;
        if (VLC_DECOMP++ <= 20530 && VLC_DECOMP > 20520 /*&& VLC_DECOMP % 10 == 0*/)
          fprintf(stderr, "vlc: %d %d %d\n", VLC_DECOMP, symbol, ff_ctx->index);
      }
      return symbol;
    }
    int get_level_prefix() {
      int symbol = 0;
      while (get() == 0) symbol++;

      // Unary
      ff_ctx->index += symbol + 1;

      {
        static int LEVEL_PREFIX_DECOMP = 0;
        if (LEVEL_PREFIX_DECOMP++ <= 260)
          fprintf(stderr, "lp: %d %d %d\n", LEVEL_PREFIX_DECOMP, symbol, ff_ctx->index);
      }

      return symbol;
    }
    unsigned int show_bits(int n) {
      const int symbol = get_bits(n);

      ff_ctx->index -= n;

      {
        static int SHOW_BITS_DECOMP = 0;
        if (SHOW_BITS_DECOMP++ <= 10) fprintf(stderr, "sb: %d %d %d\n", SHOW_BITS_DECOMP, n, symbol);
      }

      return symbol;
    }
    void skip_bits(int n) {
      ff_ctx->index += n;
    }

    void terminate() {
      finish();
    }

    void finish() {
      static int SLICE_NUM_DECOMP = 0;
      fprintf(stderr, "gb ctx: %d %d %d %d\n", ++SLICE_NUM_DECOMP, ff_ctx->index, ff_ctx->size_in_bits, ff_ctx->size_in_bits_plus8);
      // Cheat mode
      ff_ctx->size_in_bits = ff_ctx->index;
      ff_ctx->size_in_bits_plus8 = ff_ctx->size_in_bits + 8;

      out->out_bytes.assign(reinterpret_cast<const char *>(data_out.data()), data_out.size());
      out->done = true;
    }
  };

  class cabac_decoder : public generic_decoder {
   public:
    cabac_decoder(decompressor *d, CABACContext *ctx_in, const uint8_t *buf,
                  uint8_t *state_start, int size) {
      index = d->recognize_coded_block(buf, size);
      block = &d->in.block(index);
      out = &d->blocks[index];
      model = nullptr;
      this->state_start = state_start;

      if (block->has_cabac()) {
        model = &d->model;
        model->reset();
        decoder.reset(new recoded_code::decoder<const char *, uint8_t>(
            block->cabac().data(), block->cabac().data() + block->cabac().size()));
      } else if (block->has_skip_coded() && block->skip_coded()) {
        // We're skipping this block, so disable calls to our hooks.
        ctx_in->coding_hooks = nullptr;
        ctx_in->coding_hooks_opaque = nullptr;
        ::ff_reset_cabac_decoder(ctx_in, buf, size);
      } else {
        throw std::runtime_error("Expected CABAC block.");
      }
    }

    ~cabac_decoder() { assert(out->done); }

    int get(uint8_t *state_pointer) {
      int state = state_pointer - this->state_start;
      int symbol;
      if (model->coding_type == PIP_SIGNIFICANCE_EOB) {
        symbol = model->eob_symbol();
      } else {
        symbol = decoder->get([&](range_t range) {
          return model->probability_for_state(range, state);
        });
      }
      size_t billable_bytes = cabac_encoder.put(symbol, state_pointer);
      if (billable_bytes) {
        model->billable_cabac_bytes(billable_bytes);
      }
      model->update_state(symbol, state);
      return symbol;
    }

    int get_bypass() {
      int symbol = decoder->get([&](range_t range) {
        return model->probability_for_state(range, model->bypass_context);
      });
      model->update_state(symbol, model->bypass_context);
      size_t billable_bytes = cabac_encoder.put_bypass(symbol);
      if (billable_bytes) {
        model->billable_cabac_bytes(billable_bytes);
      }
      return symbol;
    }

    int get_sign_bypass() {
      int symbol = decoder->get([&](range_t range) {
        return model->probability_for_state(range, model->sign_bypass_context);
      });
      model->update_state(symbol, model->sign_bypass_context);
      size_t billable_bytes = cabac_encoder.put_bypass(symbol);
      if (billable_bytes) {
        model->billable_cabac_bytes(billable_bytes);
      }
      return symbol;
    }

    int get_terminate() {
      int symbol = decoder->get([&](range_t range) {
        return model->probability_for_state(range, model->terminate_context);
      });
      model->update_state(symbol, model->terminate_context);
      size_t billable_bytes = cabac_encoder.put_terminate(symbol);
      if (billable_bytes) {
        model->billable_cabac_bytes(billable_bytes);
      }
      if (symbol) {
        finish();
      }
      return symbol;
    }

    void finish() {
      // Omit trailing byte if it's only a stop bit.
      if (data_out.back() == 0x80) {
        data_out.pop_back();
      }
      out->out_bytes.assign(reinterpret_cast<const char *>(data_out.data()), data_out.size());
      out->done = true;
    }

   private:
    uint8_t *state_start;
  };

  h264_model *get_model() {
    return &model;
  }

 private:
  // Return a unique 8-byte string containing no zero bytes (NAL-encoding-safe).
  std::string next_surrogate_marker() {
    uint64_t n = surrogate_marker_sequence_number++;
    std::string surrogate_marker(SURROGATE_MARKER_BYTES, '\x01');
    for (int i = 0; i < (int) surrogate_marker.size(); i++) {
      surrogate_marker[i] = (n % 255) + 1;
      n /= 255;
    }
    return surrogate_marker;
  }

  std::string make_surrogate_block(const std::string &surrogate_marker, size_t size) {
    if (size < surrogate_marker.size()) {
      throw std::runtime_error("Invalid coded block size for surrogate: " + std::to_string(size));
    }
    std::string surrogate_block = surrogate_marker;
    surrogate_block.resize(size, 'X');  // NAL-encoding-safe padding.
    return surrogate_block;
  }

  int recognize_coded_block(const uint8_t *buf, int size) {
    while (!blocks[next_coded_block].coded) {
      if (next_coded_block >= read_index) {
        throw std::runtime_error("Coded block expected, but not recorded in the compressed data.");
      }
      next_coded_block++;
    }
    int index = next_coded_block++;
    // Validate the decoder init call against the coded block's size and surrogate marker.
    const Recoded::Block &block = in.block(index);
    if (block.has_cabac()) {
      if (block.size() != size) {
        fprintf(stderr, "%lu %d\n", block.size(), size);
        // throw std::runtime_error("Invalid surrogate block size.");
      }
      std::string buf_header(reinterpret_cast<const char *>(buf),
                             blocks[index].surrogate_marker.size());
      if (blocks[index].surrogate_marker != buf_header) {
        // throw std::runtime_error("Invalid surrogate marker in coded block.");
      }
    } else if (block.has_skip_coded()) {
      if (block.size() != size) {
        throw std::runtime_error("Invalid skip_coded block size.");
      }
    } else {
      throw std::runtime_error("Internal error: expected coded block.");
    }
    return index;
  }

  std::string input_filename;
  std::ostream &out_stream;

  Recoded in;
  int read_index = 0, read_offset = 0;
  std::string read_block;

  std::vector <block_state> blocks;

  // Counter used to generate surrogate markers for coded blocks.
  uint64_t surrogate_marker_sequence_number = 1;
  // Head of the coded block queue - blocks that have been produced by
  // read_packet but not yet decoded. Tail of the queue is read_index.
  int next_coded_block = 0;

  h264_model model;
};
