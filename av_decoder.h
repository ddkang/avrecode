#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <typeinfo>

extern "C" {
#include "libavcodec/avcodec.h"
#include "libavcodec/cabac.h"
#include "libavcodec/get_bits.h"
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

// Sets up a libavcodec decoder with I/O and decoding hooks.
template <typename Driver>
class av_decoder {
 public:
  av_decoder(Driver *driver, const std::string& input_filename) : driver(driver) {
    const size_t avio_ctx_buffer_size = 1024*1024;
    uint8_t *avio_ctx_buffer = static_cast<uint8_t*>( av_malloc(avio_ctx_buffer_size) );

    format_ctx = avformat_alloc_context();
    if (avio_ctx_buffer == nullptr || format_ctx == nullptr) throw std::bad_alloc();
    format_ctx->pb = avio_alloc_context(
        avio_ctx_buffer,                      // input buffer
        avio_ctx_buffer_size,                 // input buffer size
        false,                                // stream is not writable
        this,                                 // first argument for read_packet()
        read_packet,                          // read callback
        nullptr,                              // write_packet()
        nullptr);                             // seek()

    if (avformat_open_input(&format_ctx, input_filename.c_str(), nullptr, nullptr) < 0) {
      throw std::invalid_argument("Failed to initialize decoding context: " + input_filename);
    }
  }
  ~av_decoder() {
    for (size_t i = 0; i < format_ctx->nb_streams; i++) {
      avcodec_close(format_ctx->streams[i]->codec);
    }
    av_freep(&format_ctx->pb->buffer);  // May no longer be the same buffer we initially malloced.
    av_freep(&format_ctx->pb);
    avformat_close_input(&format_ctx);
  }

  // Read enough frames to display stream diagnostics. Only used by compressor,
  // because hooks are not yet set. Reads from already in-memory blocks.
  void dump_stream_info() {
    av_check( avformat_find_stream_info(format_ctx, nullptr),
        "Invalid input stream information" );
    av_dump_format(format_ctx, 0, format_ctx->filename, 0);
  }

  // Decode all video frames in the file in single-threaded mode, calling the driver's hooks.
  void decode_video() {
    auto frame = av_unique_ptr(av_frame_alloc(), av_frame_free);
    AVPacket packet;
    // TODO(ctl) add better diagnostics to error results.
    while (!av_check( av_read_frame(format_ctx, &packet), AVERROR_EOF, "Failed to read frame" )) {
      AVCodecContext *codec = format_ctx->streams[packet.stream_index]->codec;
      if (codec->codec_type == AVMEDIA_TYPE_VIDEO) {
        if (!avcodec_is_open(codec)) {
          codec->thread_count = 1;
          codec->hooks = &hooks;
          av_check( avcodec_open2(codec, avcodec_find_decoder(codec->codec_id), nullptr),
            "Failed to open decoder for stream " + std::to_string(packet.stream_index) );
        }

        int got_frame = 0;
        av_check( avcodec_decode_video2(codec, frame.get(), &got_frame, &packet),
            "Failed to decode video frame" );
      }
      av_packet_unref(&packet);
    }
  }

 private:
  // Hook stubs - wrap driver into opaque pointers.
  static int read_packet(void *opaque, uint8_t *buffer_out, int size) {
    av_decoder *self = static_cast<av_decoder*>(opaque);
    return self->driver->read_packet(buffer_out, size);
  }
  struct cabac {
    static void* init_decoder(void *opaque, CABACContext *ctx, const uint8_t *buf,
                              uint8_t *state_start, int size) {
      av_decoder *self = static_cast<av_decoder*>(opaque);
      auto *cabac_decoder = new typename Driver::cabac_decoder(self->driver, ctx, buf, state_start, size);
      self->gen_decoder.reset(cabac_decoder);
      return cabac_decoder;
    }
    static int get(void *opaque, uint8_t *state) {
      auto *self = static_cast<typename Driver::cabac_decoder*>(opaque);
      return self->get(state);
    }
    static int get_bypass(void *opaque) {
      auto *self = static_cast<typename Driver::cabac_decoder*>(opaque);
      return self->get_bypass();
    }
    static int get_sign_bypass(void *opaque) {
      auto *self = static_cast<typename Driver::cabac_decoder*>(opaque);
      return self->get_sign_bypass();
    }
    static int get_terminate(void *opaque) {
      auto *self = static_cast<typename Driver::cabac_decoder*>(opaque);
      return self->get_terminate();
    }
    static const uint8_t* skip_bytes(void *opaque, int n) {
      throw std::runtime_error("Not implemented: CABAC decoder doesn't use skip_bytes.");
    }
  };
  struct cavlc {
    static void* init_decoder(void *opaque, void* gb, const uint8_t *buf,
                              uint8_t *state_start, int size) {
      av_decoder *self = static_cast<av_decoder*>(opaque);
      auto *cavlc_decoder = new typename Driver::cavlc_decoder(self->driver, (GetBitContext*) gb, buf, size);
      self->gen_decoder.reset(cavlc_decoder);
      return cavlc_decoder;
    }
    static int get_ue_golomb(void *opaque) {
      auto *self = static_cast<typename Driver::cavlc_decoder*>(opaque);
      return self->get_ue_golomb();
    }
    static int get_ue_golomb_31(void *opaque) {
      auto *self = static_cast<typename Driver::cavlc_decoder*>(opaque);
      return self->get_ue_golomb_31();
    }
    static unsigned get_ue_golomb_long(void *opaque) {
      auto *self = static_cast<typename Driver::cavlc_decoder*>(opaque);
      return self->get_ue_golomb_long();
    }
    static int get_se_golomb(void *opaque) {
      auto *self = static_cast<typename Driver::cavlc_decoder*>(opaque);
      return self->get_se_golomb();
    }
    static unsigned int get_bits(void *opaque, int n) {
      auto *self = static_cast<typename Driver::cavlc_decoder*>(opaque);
      return self->get_bits(n);
    }
    static unsigned int get_bits1(void *opaque) {
      auto *self = static_cast<typename Driver::cavlc_decoder*>(opaque);
      return self->get_bits1();
    }
    static int get_vlc2(void *opaque, int16_t (*table)[2], int bits, int max_depth) {
      auto *self = static_cast<typename Driver::cavlc_decoder*>(opaque);
      return self->get_vlc2(table, bits, max_depth);
    }
    static int get_level_prefix(void *opaque) {
      auto *self = static_cast<typename Driver::cavlc_decoder*>(opaque);
      return self->get_level_prefix();
    }
    static unsigned int show_bits(void *opaque, int n) {
      auto *self = static_cast<typename Driver::cavlc_decoder*>(opaque);
      return self->show_bits(n);
    }
    static void skip_bits(void *opaque, int n) {
      auto *self = static_cast<typename Driver::cavlc_decoder*>(opaque);
      return self->skip_bits(n);
    }

    static void terminate(void *opaque) {
      auto *self = static_cast<typename Driver::cavlc_decoder*>(opaque);
      self->terminate();
    }
  };
  struct model_hooks {
    static void frame_spec(void *opaque, int frame_num, int mb_width, int mb_height) {
      auto *self = static_cast<av_decoder*>(opaque)->driver->get_model();
      self->update_frame_spec(frame_num, mb_width, mb_height);
    }
    static void mb_xy(void *opaque, int x, int y) {
      auto *self = static_cast<av_decoder*>(opaque)->driver->get_model();
      self->mb_coord.mb_x = x;
      self->mb_coord.mb_y = y;
      // LOG_NEIGHBORS("%d %d\n", x, y);
    }
    static void begin_sub_mb(void *opaque, int cat, int scan8index, int max_coeff, int is_dc, int chroma422) {
      auto *self = static_cast<av_decoder*>(opaque)->driver->get_model();
      self->sub_mb_cat = cat;
      self->mb_coord.scan8_index = scan8index;
      self->sub_mb_size = max_coeff;
      self->sub_mb_is_dc = is_dc;
      self->sub_mb_chroma422 = chroma422;

      TOTAL_NUM++;
      if (max_coeff == 4)
        NUM_2X2++;
      else if (max_coeff == 15)
        NUM_WEIRD++;
      else if (max_coeff == 16 || max_coeff == 15)
        NUM_4X4++;
      else if (max_coeff == 64)
        NUM_8X8++;
      else {
        fprintf(stderr, "%d\n", max_coeff);
        assert(false);
      }
    }
    static void end_sub_mb(void *opaque, int cat, int scan8index, int max_coeff, int is_dc, int chroma422) {
      auto *self = static_cast<av_decoder*>(opaque)->driver->get_model();
      assert(self->sub_mb_cat == cat);
      assert(self->mb_coord.scan8_index == scan8index);
      assert(self->sub_mb_size == max_coeff);
      assert(self->sub_mb_is_dc == is_dc);
      assert(self->sub_mb_chroma422 == chroma422);
      self->print_coeffs();
      self->sub_mb_cat = -1;
      self->mb_coord.scan8_index = -1;
      self->sub_mb_size = -1;
      self->sub_mb_is_dc = 0;
      self->sub_mb_chroma422 = 0;
    }
    static void begin_coding_type(void *opaque, CodingType ct,
                                    int zigzag_index, int param0, int param1) {
      auto self = static_cast<av_decoder*>(opaque)->gen_decoder.get();
      self->begin_coding_type(ct, zigzag_index, param0, param1);
    }
    static void end_coding_type(void *opaque, CodingType ct) {
      auto self = static_cast<av_decoder*>(opaque)->gen_decoder.get();
      self->end_coding_type(ct);
    }
    static void copy_coefficients(void *opaque, int16_t *block, int max_coeff) {
      auto self = static_cast<av_decoder*>(opaque)->gen_decoder.get();
      self->copy_coefficients(block, max_coeff);
    }
    static void set_mb_type(void *opaque, int mb_type) {
      auto self = static_cast<av_decoder*>(opaque)->gen_decoder.get();
      self->set_mb_type(mb_type);
    }
  };
  Driver *driver;
  AVFormatContext *format_ctx;
  AVCodecHooks hooks = { this, {
      cabac::init_decoder,
      cabac::get,
      cabac::get_bypass,
      cabac::get_sign_bypass,
      cabac::get_terminate,
      cabac::skip_bytes,
    },
    {
      cavlc::init_decoder,
      cavlc::get_ue_golomb,
      cavlc::get_ue_golomb_31,
      cavlc::get_ue_golomb_long,
      cavlc::get_se_golomb,
      cavlc::get_bits,
      cavlc::get_bits1,
      cavlc::get_vlc2,
      cavlc::get_level_prefix,
      cavlc::show_bits,
      cavlc::skip_bits,
      cavlc::terminate,
    },
    {
      model_hooks::frame_spec,
      model_hooks::mb_xy,
      model_hooks::begin_sub_mb,
      model_hooks::end_sub_mb,
      model_hooks::begin_coding_type,
      model_hooks::end_coding_type,
      model_hooks::copy_coefficients,
      model_hooks::set_mb_type,
    },
  };
  std::unique_ptr<typename Driver::generic_decoder> gen_decoder;
};