extern "C" {
#include "libavcodec/avcodec.h"
#include "libavcodec/cabac.h"
#include "libavcodec/coding_hooks.h"
#include "libavformat/avformat.h"
#include "libavformat/avio.h"
#include "libavutil/error.h"
#include "libavutil/file.h"
}

#include <deque>
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

  /*
   * This is an extremely annoying hack to keep development moving.
   * The compressor, upon construction, does not know whether it will be
   * decoding CAVLC or CABAC, so it doesn't know which model to construct.
   * Construct upon the first time asked, and not any time after.
   */
  void make_model(bool cabac) {
    if (!model) {
      if (cabac)
        model = std::make_unique<cabac_model>();
      else
        model = std::make_unique<cavlc_model>();
    }
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
                                 [&](estimator *e, int *symbol, int context) {
                                   *symbol = decoder->get([&](range_t range) {
                                     return model->probability_for_model_key(range, e);
                                   });
                                   model->update_state_for_model_key(*symbol, context, e);
                                 });
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
      EstimatorContext::copy_coefficients(block, max_coeff);
    }

    void set_mb_type(int mb_type) {
      if (!model) return;
      EstimatorContext::set_mb_type(mb_type);
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
      d->make_model(false);
      index = d->recognize_coded_block(buf, size);
      block = &d->in.block(index);
      out = &d->blocks[index];
      model = nullptr;

      if (block->has_cabac()) {
        model = d->get_model();
        model->reset();
        decoder.reset(new recoded_code::decoder<const char *, uint8_t>(
            block->cabac().data(), block->cabac().data() + block->cabac().size()));

        std::bitset<32> gb_size_bits(0), gb_index(0);
        for (int i = 0; i < gb_size_bits.size(); i++) gb_size_bits[i] = read1_no_put();
        for (int i = 0; i < gb_index.size(); i++) gb_index[i] = read1_no_put();
        assert(gb_index.to_ulong() == ctx_in->index);

        auto a = ctx_in->cavlc_hooks;
        auto b = ctx_in->cavlc_hooks_opaque;
        ::init_get_bits(ctx_in, ctx_in->buffer, gb_size_bits.to_ulong());
        ctx_in->cavlc_hooks = a;
        ctx_in->cavlc_hooks_opaque = b;

        ctx_in->index = gb_index.to_ulong();

        left_to_write = (-ctx_in->index) & 7;
      } else if (block->has_skip_coded() && block->skip_coded()) {
        // We're skipping this block, so disable calls to our hooks.
        ctx_in->cavlc_hooks = nullptr;
        ctx_in->cavlc_hooks_opaque = nullptr;
      } else {
        throw std::runtime_error("Expected CAVLC block.");
      }
      ff_ctx = ctx_in;
    }
    GetBitContext *ff_ctx;
    int left_to_write = 7;

    ~cavlc_decoder() { assert(out->done); }

    size_t put_single(int symbol) {
      if (left_to_write-- > 0) return 0;
      size_t billable_bytes = cavlc_encoder.put(symbol);
      if (billable_bytes) {
        model->billable_cabac_bytes(billable_bytes);
      }
      return billable_bytes;
    }

    int read1_no_put() {
      const int state = 0;
      const int symbol = decoder->get([&](range_t range) {
        return model->probability_for_state(range, state);
      });
      model->update_state(symbol, state);
      return symbol;
    }
    int read1() {
      const int state = 0;
      const int symbol = decoder->get([&](range_t range) {
        return model->probability_for_state(range, state);
      });
      put_single(symbol);
      model->update_state(symbol, state);
      return symbol;
    }
    int read(int n) {
      int symbol = 0;
      for (int i = 0; i < n; i++)
        symbol = (symbol << 1) + read1();
      return symbol;
    }


    unsigned decode_golomb(bool print) {
      std::bitset<32> bits(0);
      int zeros = 0;
      while (0 == read1() && zeros < 32) zeros++;

      bits[zeros] = 1;
      for (int i = 0; i < zeros; i++)
        bits[zeros - i - 1] = read1();
      int symbol = bits.to_ulong() - 1;

      const int len = zeros * 2 + 1;
      ff_ctx->index += len;
      return symbol;
    }
    unsigned decode_golomb() {
      return decode_golomb(true);
    }

    int get_ue_golomb() {
      return decode_golomb();
    }
    int get_ue_golomb_31() {
      // Truncated unary
      int symbol = 0;
      if (read1_no_put()) symbol = 0;
      else if (read1_no_put()) symbol = 1;
      else if (read1_no_put()) symbol = 2;
      else symbol = 3;

      const int len = av_log2(symbol + 1) * 2 + 1;
      const std::bitset<32> bits(symbol + 1); // The last len bits
      for (int i = len - 1; i >= 0; i--)
        put_single(bits[i]);
      ff_ctx->index += len;

      return symbol;
      // return decode_golomb();
    }
    unsigned get_ue_golomb_long() {
      return decode_golomb();
    }
    int get_se_golomb() {
      const int base_symbol = decode_golomb();
      const int symbol = base_symbol % 2 == 0 ? -base_symbol / 2 : base_symbol / 2 + 1;
      return symbol;
    }
    unsigned int get_bits(int n) {
      const int symbol = read(n);
      ff_ctx->index += n;
      return symbol;
    }
    unsigned int get_bits1() {
      ff_ctx->index += 1;
      return read1();
    }
    // We prevent checking for bits == 0 because this indicates an invalid VLC code
    int get_vlc2(int16_t (*table)[2], const int read_bits, const int max_depth) {
      unsigned int index = 0;
      for (int bits = 0; bits < read_bits; bits++) {
        if (bits != 0 && table[index][1] == bits)
          break;
        index |= read1() << (read_bits - bits - 1);
        ff_ctx->index += 1; // FIXME: optimize
      }
      const int code = table[index][0];
      const int n    = table[index][1];
      if (max_depth > 1 && n < 0) {
        index = code;
        const int nb_bits = -n;
        for (int bits = 0; bits < nb_bits; bits++) {
          if (bits != 0 && table[index][1] == bits)
            break;
          index += read1() << (nb_bits - bits - 1);
          ff_ctx->index += 1; // FIXME: optimize
        }

        return table[index][0];
      }
      return code;
    }
    int get_level_prefix() {
      int symbol = 0;
      while (read1() == 0) symbol++;
      // Unary
      ff_ctx->index += symbol + 1;
      return symbol;
    }

    unsigned int last_show_bits = 0;
    unsigned int show_bits(int n) {
      const unsigned int symbol = last_show_bits = read(n);
      ff_ctx->index += n;
      return symbol;
    }
    unsigned int recode_show_bits(const int8_t table[256][2], int n) {
      unsigned int symbol = 0;
      for (int bits = 0; bits < n; bits++) {
        if (table[symbol][1] == bits)
          break;
        symbol |= read1() << (n - bits - 1);
        ff_ctx->index += 1; // FIXME: optimize
      }

      return symbol;
    }
    void skip_bits(int n) {
    }

    void terminate() {
      finish();
    }

    void finish() {
      assert(ff_ctx->size_in_bits == ff_ctx->index);

      out->out_bytes.assign(reinterpret_cast<const char *>(data_out.data()), data_out.size());
      out->done = true;
    }
  };

  class cabac_decoder : public generic_decoder {
   public:
    cabac_decoder(decompressor *d, CABACContext *ctx_in, const uint8_t *buf,
                  uint8_t *state_start, int size) {
      d->make_model(true);
      index = d->recognize_coded_block(buf, size);
      block = &d->in.block(index);
      out = &d->blocks[index];
      model = nullptr;
      this->state_start = state_start;

      if (block->has_cabac()) {
        model = d->get_model();
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
        symbol = EstimatorContext::eob_symbol();
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
        return model->probability_for_state(range, EstimatorContext::bypass_context);
      });
      model->update_state(symbol, EstimatorContext::bypass_context);
      size_t billable_bytes = cabac_encoder.put_bypass(symbol);
      if (billable_bytes) {
        model->billable_cabac_bytes(billable_bytes);
      }
      return symbol;
    }

    int get_sign_bypass() {
      int symbol = decoder->get([&](range_t range) {
        return model->probability_for_state(range, EstimatorContext::sign_bypass_context);
      });
      model->update_state(symbol, EstimatorContext::sign_bypass_context);
      size_t billable_bytes = cabac_encoder.put_bypass(symbol);
      if (billable_bytes) {
        model->billable_cabac_bytes(billable_bytes);
      }
      return symbol;
    }

    int get_terminate() {
      int symbol = decoder->get([&](range_t range) {
        return model->probability_for_state(range, EstimatorContext::terminate_context);
      });
      model->update_state(symbol, EstimatorContext::terminate_context);
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
    return model.get();
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
        // fprintf(stderr, "%lu %d\n", block.size(), size);
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

  std::unique_ptr<h264_model> model;
};