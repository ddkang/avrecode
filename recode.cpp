/* -*-mode:c++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
#include <fstream>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <sstream>
#include <typeinfo>
#include <vector>

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
#include "av_decoder.h"
#include "h264_model.h"
#include "h264_symbol.h"
#include "compressor.h"
#include "decompressor.h"

int test_reverse_scan8() {
  for (size_t i = 0; i < sizeof(scan_8) / sizeof(scan_8[0]); ++i) {
    auto a = reverse_scan_8[scan_8[i] >> 3][scan_8[i] & 7];
    assert(a.neighbor_left == false && a.neighbor_up == false);
    assert(a.scan8_index == i);
    if (a.scan8_index != i) {
      return 1;
    }
  }
  for (int i = 0; i < 16; ++i) {
    assert(zigzag16[unzigzag16[i]] == i);
    assert(unzigzag16[zigzag16[i]] == i);
  }
  return 0;
}

int make_sure_reverse_scan8 = test_reverse_scan8();


int roundtrip(const std::string &input_filename, std::ostream *out) {
  std::stringstream original, compressed, decompressed;
  original << std::ifstream(input_filename).rdbuf();
  compressor c(input_filename, compressed);
  c.run();
  decompressor d(input_filename, compressed.str(), decompressed);
  d.run();

  if (original.str() == decompressed.str()) {
    if (out) {
      (*out) << compressed.str();
    }
    double ratio = compressed.str().size() * 1.0 / original.str().size();

    Recoded compressed_proto;
    compressed_proto.ParseFromString(compressed.str());
    int proto_block_bytes = 0;
    for (const auto &block : compressed_proto.block()) {
      proto_block_bytes += block.literal().size() + block.cabac().size();
    }
    double proto_overhead = (compressed.str().size() - proto_block_bytes) * 1.0 / compressed.str().size();

    std::cout << "Compress-decompress roundtrip succeeded:" << std::endl;
    std::cout << " compression ratio: " << ratio * 100. << "%" << std::endl;
    std::cout << " protobuf overhead: " << proto_overhead * 100. << "%" << std::endl;
    std::cout << ratio << std::endl;
    return 0;
  } else {
    std::cerr << "Compress-decompress roundtrip failed." << std::endl;
    return 1;
  }
}


int main(int argc, char **argv) {
  av_register_all();

  if (argc < 3 || argc > 4) {
    std::cerr << "Usage: " << argv[0] << " [compress|decompress|roundtrip] <input> [output]" << std::endl;
    return 1;
  }
  std::string command = argv[1];
  std::string input_filename = argv[2];
  std::ofstream out_file;
  if (argc > 3) {
    out_file.open(argv[3]);
  }

  try {
    if (command == "compress") {
      std::stringstream original, compressed;
      original << std::ifstream(input_filename).rdbuf();
      // compressor c(input_filename, out_file.is_open() ? out_file : std::cout);
      compressor c(input_filename, compressed);
      c.run();
      double ratio = compressed.str().size() * 1.0 / original.str().size();
      std::cout << ratio << std::endl;
    } else if (command == "decompress") {
      decompressor d(input_filename, out_file.is_open() ? out_file : std::cout);
      d.run();
    } else if (command == "roundtrip") {
      return roundtrip(input_filename, out_file.is_open() ? &out_file : nullptr);
    } else {
      throw std::invalid_argument("Unknown command: " + command);
    }
  } catch (const std::exception &e) {
    std::cerr << "Exception (" << typeid(e).name() << "): " << e.what() << std::endl;
    return 1;
  }
  return 0;
}
