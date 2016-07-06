#ifndef _BLOCK_H_
#define _BLOCK_H_

struct Block {
    int16_t residual[(3 * (16 + 1)) * 16];
    int16_t mv_x[4][4];
    int16_t mv_y[4][4];
};
struct BlockMeta{
    int32_t rem_pred_mode[16];
    int32_t prev_pred_mode[16];
    uint8_t sub_mb_type[4];
    uint8_t refIdx[4];
    uint8_t cbp;
    uint8_t mb_type;
    uint8_t lumai16x16mode;
    uint8_t chromai8x8mode;
    uint8_t last_mb_qp;
    uint8_t luma_qp;
    bool is_8x8;
    bool coded;
    uint8_t num_nonzeros[(3 * (16 + 1))];
};
#endif

