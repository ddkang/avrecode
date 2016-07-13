#ifndef _BLOCK_H_
#define _BLOCK_H_

struct Block {
    int16_t coeffs[(3 * (16 + 1)) * 16 + 64];
    int16_t residual[(3 * (16 + 1)) * 16];
};
struct BlockMeta {
    uint8_t sub_mb_size;
    bool is_8x8;
    bool coded;
    uint8_t num_nonzeros[(3 * (16 + 1))];
};
#endif

