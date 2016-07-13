#ifndef _BLOCK_H_
#define _BLOCK_H_

struct Block {
    int16_t coeffs[(3 * (16 + 1)) * 16 + 64];
    int16_t residual[(3 * (16 + 1)) * 16];
};
struct BlockMeta {
    uint8_t intra4x4_pred_mode[16];
    uint8_t sub_mb_size;
    uint8_t cbp_luma;
    bool is_8x8;
    bool coded;
    uint8_t num_nonzeros[(3 * (16 + 1))];
};
#endif

