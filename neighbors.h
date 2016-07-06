#include "recode.h"

#pragma once


struct CoordSub {
  int mb;
  int scan8_index;
};

bool get_neighbor_sub_mb_4x4(CoordSub  inp, CoordSub *out,
                             int left_shift, int above_shift) {
  int mb = inp.mb;
  int scan8_index = inp.scan8_index;
  out->mb = inp.mb;
  out->scan8_index = inp.scan8_index;
  if (scan8_index >= 16 * 3) {
    if (mb > 0) {
      out->mb -= 1;
      return true;
    }
    return false;
  }
  int scan8 = scan_8[scan8_index];
  auto neighbor = reverse_scan_8[(scan8 >> 3) + above_shift][(scan8 & 7) + left_shift];
  bool neighbor_in = (left_shift == 0) ? neighbor.neighbor_up : neighbor.neighbor_left;
  if (neighbor_in) {
    if (mb == 0) {
      return false;
    } else {
      --mb;
    }
  }
  out->scan8_index = neighbor.scan8_index;
  out->mb = mb;
  return true;
}

bool get_neighbor_sub_mb(bool above, int sub_mb_size,
                         CoefficientCoord input,
                         CoefficientCoord *output) {
  int left_shift = (above ? 0 : -1);
  int above_shift = (above ? -1 : 0);

  int inp_mb = above ? input.mb_y : input.mb_x;
  CoordSub inp = {inp_mb, input.scan8_index};
  CoordSub out = {0, 0};

  bool result = get_neighbor_sub_mb_4x4(inp, &out, left_shift, above_shift);

  output->mb_x = above ? input.mb_x : out.mb;
  output->mb_y = above ? out.mb : input.mb_y;
  output->scan8_index = out.scan8_index;
  output->zigzag_index = input.zigzag_index;

  if (sub_mb_size >= 32) {
    switch (input.scan8_index) {
      case 0:
        output->scan8_index = 0;
        break;
      case 4:
        output->scan8_index = above ? 4 : 0;
        break;
      case 8:
        output->scan8_index = above ? 0 : 8;
        break;
      case 12:
        output->scan8_index = above ? 4 : 8;
        break;
      default:
        assert(false);
    }
    output->scan8_index /= 4;
    output->scan8_index *= 4; // round down to the nearest multiple of 4
  }

  return result;
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
    dim = 8;
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

