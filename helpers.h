extern "C" {
#include "libavcodec/mpegutils.h"
}

#pragma once

enum MB_TYPE {
  MB_INTRA4x4,
  MB_INTRA16x16,
  MB_INTRA8x8,
  MB_16x16,
  MB_16x8,
  MB_8x16,
  MB_8x8,
  MB_SKIP,
  MB_NONE,
};

int parse_ff_mb_type(int ff_mb_type) {
  if (IS_INTRA4x4(ff_mb_type))
    return MB_INTRA4x4;
  if (IS_INTRA16x16(ff_mb_type))
    return MB_INTRA16x16;
  if (IS_16X16(ff_mb_type))
    return MB_16x16;
  if (IS_16X8(ff_mb_type))
    return MB_16x8;
  if (IS_8X16(ff_mb_type))
    return MB_8x16;
  if (IS_8X8(ff_mb_type))
    return MB_8x8;
  if (IS_SKIP(ff_mb_type))
    return MB_SKIP;
  // assert(false);

  if (ff_mb_type != 0)
    assert(false);
  return MB_NONE;
}
