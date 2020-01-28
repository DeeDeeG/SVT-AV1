/*
* Copyright(c) 2019 Intel Corporation
* SPDX - License - Identifier: BSD - 2 - Clause - Patent
*/

#ifndef EbInterPrediction_h
#define EbInterPrediction_h

//#include "EbModeDecision.h"
#include "EbBlockStructures.h"
#include "EbAv1Structs.h"
#include "EbObject.h"
#include "EbMcp.h"
#include "filter.h"
#include "convolve.h"

#ifdef __cplusplus
extern "C" {
#endif

#define REF_SCALE_SHIFT 14
#define REF_NO_SCALE (1 << REF_SCALE_SHIFT)
#define REF_INVALID_SCALE -1

#define RS_SUBPEL_BITS 6
#define RS_SUBPEL_MASK ((1 << RS_SUBPEL_BITS) - 1)
#define RS_SCALE_SUBPEL_BITS 14
#define RS_SCALE_SUBPEL_MASK ((1 << RS_SCALE_SUBPEL_BITS) - 1)
#define RS_SCALE_EXTRA_BITS (RS_SCALE_SUBPEL_BITS - RS_SUBPEL_BITS)
#define RS_SCALE_EXTRA_OFF (1 << (RS_SCALE_EXTRA_BITS - 1))

    extern DECLARE_ALIGNED(256, const InterpKernel, sub_pel_filters_8[SUBPEL_SHIFTS]);
    extern DECLARE_ALIGNED(256, const InterpKernel, sub_pel_filters_4[SUBPEL_SHIFTS]);
    extern DECLARE_ALIGNED(256, const InterpKernel, sub_pel_filters_8sharp[SUBPEL_SHIFTS]);
    extern DECLARE_ALIGNED(256, const InterpKernel, sub_pel_filters_8smooth[SUBPEL_SHIFTS]);
    extern DECLARE_ALIGNED(256, const InterpKernel, bilinear_filters[SUBPEL_SHIFTS]);
    extern DECLARE_ALIGNED(256, const InterpKernel, sub_pel_filters_4smooth[SUBPEL_SHIFTS]);

    typedef struct SubpelParams {
        int32_t xs;
        int32_t ys;
        int32_t subpel_x;
        int32_t subpel_y;
    } SubpelParams;

typedef uint8_t *WedgeMasksType[MAX_WEDGE_TYPES];
static WedgeMasksType wedge_masks[BlockSizeS_ALL][2];
// Angles are with respect to horizontal anti-clockwise
typedef enum WedgeDirectionType
{
    WEDGE_HORIZONTAL = 0,
    WEDGE_VERTICAL = 1,
    WEDGE_OBLIQUE27 = 2,
    WEDGE_OBLIQUE63 = 3,
    WEDGE_OBLIQUE117 = 4,
    WEDGE_OBLIQUE153 = 5,
    WEDGE_DIRECTIONS
} WedgeDirectionType;

static INLINE void clamp_mv(MV *mv, int32_t min_col, int32_t max_col, int32_t min_row,
                            int32_t max_row) {
    mv->col = (int16_t)clamp(mv->col, min_col, max_col);
    mv->row = (int16_t)clamp(mv->row, min_row, max_row);
}

// 3-tuple: {direction, x_offset, y_offset}
typedef struct WedgeCodeType
{
    WedgeDirectionType direction;
    int32_t x_offset;
    int32_t y_offset;
} WedgeCodeType;

typedef struct WedgeParamsType
{
    int32_t bits;
    const WedgeCodeType *codebook;
    uint8_t *signflip;
    WedgeMasksType *masks;
} WedgeParamsType;


DECLARE_ALIGNED(16, static uint8_t, wedge_signflip_lookup[BlockSizeS_ALL][MAX_WEDGE_TYPES]) = {
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },  // not used
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },  // not used
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },  // not used
        { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, },
        { 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, },
        { 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, },
        { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, },
        { 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, },
        { 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, },
        { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, },
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },  // not used
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },  // not used
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },  // not used
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },  // not used
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },  // not used
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },  // not used
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },  // not used
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },  // not used
        { 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, },
        { 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, },
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },  // not used
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },  // not used
};


static const WedgeCodeType wedge_codebook_16_hgtw[16] = {
        { WEDGE_OBLIQUE27, 4, 4 }, { WEDGE_OBLIQUE63, 4, 4 },
        { WEDGE_OBLIQUE117, 4, 4 }, { WEDGE_OBLIQUE153, 4, 4 },
        { WEDGE_HORIZONTAL, 4, 2 }, { WEDGE_HORIZONTAL, 4, 4 },
        { WEDGE_HORIZONTAL, 4, 6 }, { WEDGE_VERTICAL, 4, 4 },
        { WEDGE_OBLIQUE27, 4, 2 }, { WEDGE_OBLIQUE27, 4, 6 },
        { WEDGE_OBLIQUE153, 4, 2 }, { WEDGE_OBLIQUE153, 4, 6 },
        { WEDGE_OBLIQUE63, 2, 4 }, { WEDGE_OBLIQUE63, 6, 4 },
        { WEDGE_OBLIQUE117, 2, 4 }, { WEDGE_OBLIQUE117, 6, 4 },
};

static const WedgeCodeType wedge_codebook_16_hltw[16] = {
        { WEDGE_OBLIQUE27, 4, 4 }, { WEDGE_OBLIQUE63, 4, 4 },
        { WEDGE_OBLIQUE117, 4, 4 }, { WEDGE_OBLIQUE153, 4, 4 },
        { WEDGE_VERTICAL, 2, 4 }, { WEDGE_VERTICAL, 4, 4 },
        { WEDGE_VERTICAL, 6, 4 }, { WEDGE_HORIZONTAL, 4, 4 },
        { WEDGE_OBLIQUE27, 4, 2 }, { WEDGE_OBLIQUE27, 4, 6 },
        { WEDGE_OBLIQUE153, 4, 2 }, { WEDGE_OBLIQUE153, 4, 6 },
        { WEDGE_OBLIQUE63, 2, 4 }, { WEDGE_OBLIQUE63, 6, 4 },
        { WEDGE_OBLIQUE117, 2, 4 }, { WEDGE_OBLIQUE117, 6, 4 },
};

static const WedgeCodeType wedge_codebook_16_heqw[16] = {
        { WEDGE_OBLIQUE27, 4, 4 }, { WEDGE_OBLIQUE63, 4, 4 },
        { WEDGE_OBLIQUE117, 4, 4 }, { WEDGE_OBLIQUE153, 4, 4 },
        { WEDGE_HORIZONTAL, 4, 2 }, { WEDGE_HORIZONTAL, 4, 6 },
        { WEDGE_VERTICAL, 2, 4 }, { WEDGE_VERTICAL, 6, 4 },
        { WEDGE_OBLIQUE27, 4, 2 }, { WEDGE_OBLIQUE27, 4, 6 },
        { WEDGE_OBLIQUE153, 4, 2 }, { WEDGE_OBLIQUE153, 4, 6 },
        { WEDGE_OBLIQUE63, 2, 4 }, { WEDGE_OBLIQUE63, 6, 4 },
        { WEDGE_OBLIQUE117, 2, 4 }, { WEDGE_OBLIQUE117, 6, 4 },
};

static const WedgeParamsType wedge_params_lookup[BlockSizeS_ALL] = {
        { 0, NULL, NULL, NULL },
        { 0, NULL, NULL, NULL },
        { 0, NULL, NULL, NULL },
        { 4, wedge_codebook_16_heqw, wedge_signflip_lookup[BLOCK_8X8],
                wedge_masks[BLOCK_8X8] },
        { 4, wedge_codebook_16_hgtw, wedge_signflip_lookup[BLOCK_8X16],
                wedge_masks[BLOCK_8X16] },
        { 4, wedge_codebook_16_hltw, wedge_signflip_lookup[BLOCK_16X8],
                wedge_masks[BLOCK_16X8] },
        { 4, wedge_codebook_16_heqw, wedge_signflip_lookup[BLOCK_16X16],
                wedge_masks[BLOCK_16X16] },
        { 4, wedge_codebook_16_hgtw, wedge_signflip_lookup[BLOCK_16X32],
                wedge_masks[BLOCK_16X32] },
        { 4, wedge_codebook_16_hltw, wedge_signflip_lookup[BLOCK_32X16],
                wedge_masks[BLOCK_32X16] },
        { 4, wedge_codebook_16_heqw, wedge_signflip_lookup[BLOCK_32X32],
                wedge_masks[BLOCK_32X32] },
        { 0, NULL, NULL, NULL },
        { 0, NULL, NULL, NULL },
        { 0, NULL, NULL, NULL },
        { 0, NULL, NULL, NULL },
        { 0, NULL, NULL, NULL },
        { 0, NULL, NULL, NULL },
        { 0, NULL, NULL, NULL },
        { 0, NULL, NULL, NULL },
        { 4, wedge_codebook_16_hgtw, wedge_signflip_lookup[BLOCK_8X32],
                wedge_masks[BLOCK_8X32] },
        { 4, wedge_codebook_16_hltw, wedge_signflip_lookup[BLOCK_32X8],
                wedge_masks[BLOCK_32X8] },
        { 0, NULL, NULL, NULL },
        { 0, NULL, NULL, NULL },
};

    struct ModeDecisionContext;

    typedef struct InterPredictionContext {
        EbDctor                               dctor;
        MotionCompensationPredictionContext  *mcp_context;
    } InterPredictionContext;

    void svt_inter_predictor(const uint8_t *src, int32_t src_stride,
        uint8_t *dst, int32_t dst_stride, const SubpelParams *subpel_params,
        const ScaleFactors *sf, int32_t w, int32_t h, ConvolveParams *conv_params,
        InterpFilters interp_filters, int32_t is_intrabc);

    void svt_highbd_inter_predictor(const uint16_t *src, int32_t src_stride,
        uint16_t *dst, int32_t dst_stride, const SubpelParams *subpel_params,
        const ScaleFactors *sf, int32_t w, int32_t h, ConvolveParams *conv_params,
        InterpFilters interp_filters, int32_t is_intrabc, int32_t bd);


    void av1_dist_wtd_comp_weight_assign(
        SeqHeader *seq_header,
        int cur_frame_index,
        int bck_frame_index,
        int fwd_frame_index,
        int compound_idx,
        int order_idx,
        int *fwd_offset, int *bck_offset,
        int *use_dist_wtd_comp_avg,
        int is_compound);

    void build_masked_compound_no_round(uint8_t *dst, int dst_stride,
        const CONV_BUF_TYPE *src0, int src0_stride,
        const CONV_BUF_TYPE *src1, int src1_stride,
        const InterInterCompoundData *const comp_data, uint8_t *seg_mask,
        BlockSize sb_type, int h, int w, ConvolveParams *conv_params,
        uint8_t bd);

    void av1_get_convolve_filter_params(uint32_t interp_filters,
        InterpFilterParams *params_x, InterpFilterParams *params_y,
        int32_t w, int32_t h);

    /* Mapping of interintra to intra mode for use in the intra component */
    static const PredictionMode interintra_to_intra_mode[INTERINTRA_MODES] = {
      DC_PRED, V_PRED, H_PRED, SMOOTH_PRED
    };
    static INLINE int is_interintra_wedge_used(BlockSize sb_type) {
        return wedge_params_lookup[sb_type].bits > 0;
    }

    void combine_interintra(InterIntraMode mode,
        int8_t use_wedge_interintra, int wedge_index,
        int wedge_sign, BlockSize bsize,
        BlockSize plane_bsize, uint8_t *comppred,
        int compstride, const uint8_t *interpred,
        int interstride, const uint8_t *intrapred,
        int intrastride);

    void combine_interintra_highbd(
        InterIntraMode mode, uint8_t use_wedge_interintra, uint8_t wedge_index,
        uint8_t wedge_sign, BlockSize bsize, BlockSize plane_bsize,
        uint8_t *comppred8, int compstride, const uint8_t *interpred8,
        int interstride, const uint8_t *intrapred8, int intrastride, int bd);


    void av1_setup_scale_factors_for_frame(ScaleFactors *sf, int other_w,
        int other_h, int this_w, int this_h);

    static INLINE int av1_is_valid_scale(const struct ScaleFactors *sf) {
        return sf->x_scale_fp != REF_INVALID_SCALE &&
            sf->y_scale_fp != REF_INVALID_SCALE;
    }
    static INLINE int av1_is_scaled(const struct ScaleFactors *sf) {
        return av1_is_valid_scale(sf) &&
            (sf->x_scale_fp != REF_NO_SCALE || sf->y_scale_fp != REF_NO_SCALE);
    }
    static INLINE int valid_ref_frame_size(int ref_width, int ref_height,
        int this_width, int this_height)
    {
        return 2 * this_width >= ref_width && 2 * this_height >= ref_height &&
            this_width <= 16 * ref_width && this_height <= 16 * ref_height;
    }
    MV32 av1_scale_mv(const MV *mvq4, int x, int y,
        const ScaleFactors *sf);

static INLINE int get_wedge_bits_lookup(BlockSize sb_type) {
    return wedge_params_lookup[sb_type].bits;
}

static INLINE const uint8_t *av1_get_contiguous_soft_mask(int wedge_index, int wedge_sign,
                                                          BlockSize sb_type) {
    return wedge_params_lookup[sb_type].masks[wedge_sign][wedge_index];
}

void build_smooth_interintra_mask(uint8_t *mask, int stride, BlockSize plane_bsize,
                                  InterIntraMode mode);

void highbd_convolve_2d_for_intrabc(const uint16_t *src, int src_stride, uint16_t *dst,
                                    int dst_stride, int w, int h, int subpel_x_q4,
                                    int subpel_y_q4, ConvolveParams *conv_params, int bd);

void convolve_2d_for_intrabc(const uint8_t *src, int src_stride, uint8_t *dst,
                             int dst_stride, int w, int h, int subpel_x_q4, int subpel_y_q4,
                             ConvolveParams *conv_params);

        extern aom_highbd_convolve_fn_t convolve_hbd[/*sub_x*/2][/*sub_y*/2][/*bi*/2];
    extern AomConvolveFn convolve[/*sub_x*/2][/*sub_y*/2][/*bi*/2];

#ifdef __cplusplus
}
#endif
#endif //EbInterPrediction_h
