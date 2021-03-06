/*
 * Copyright (c) 2019, Alliance for Open Media. All rights reserved
 *
 * This source code is subject to the terms of the BSD 2 Clause License and
 * the Alliance for Open Media Patent License 1.0. If the BSD 2 Clause License
 * was not distributed with this source code in the LICENSE file, you can
 * obtain it at www.aomedia.org/license/software. If the Alliance for Open
 * Media Patent License 1.0 was not distributed with this source code in the
 * PATENTS file, you can obtain it at www.aomedia.org/license/patent.
 */

#include <assert.h>
#include <smmintrin.h>

#include "EbDefinitions.h"
#include "EbTemporalFiltering_constants.h"
#include "EbTemporalFiltering_sse4.h"

// Read in 8 pixels from a and b as 8-bit unsigned integers, compute the
// difference squared, and store as unsigned 16-bit integer to dst.
static INLINE void store_dist_8(const uint8_t *a, const uint8_t *b, uint16_t *dst) {
    const __m128i a_reg = _mm_loadl_epi64((const __m128i *)a);
    const __m128i b_reg = _mm_loadl_epi64((const __m128i *)b);

    const __m128i a_first = _mm_cvtepu8_epi16(a_reg);
    const __m128i b_first = _mm_cvtepu8_epi16(b_reg);

    __m128i dist_first;

    dist_first = _mm_sub_epi16(a_first, b_first);
    dist_first = _mm_mullo_epi16(dist_first, dist_first);

    _mm_storeu_si128((__m128i *)dst, dist_first);
}

static INLINE void store_dist_16(const uint8_t *a, const uint8_t *b, uint16_t *dst) {
    const __m128i zero  = _mm_setzero_si128();
    const __m128i a_reg = _mm_loadu_si128((const __m128i *)a);
    const __m128i b_reg = _mm_loadu_si128((const __m128i *)b);

    const __m128i a_first  = _mm_cvtepu8_epi16(a_reg);
    const __m128i a_second = _mm_unpackhi_epi8(a_reg, zero);
    const __m128i b_first  = _mm_cvtepu8_epi16(b_reg);
    const __m128i b_second = _mm_unpackhi_epi8(b_reg, zero);

    __m128i dist_first, dist_second;

    dist_first  = _mm_sub_epi16(a_first, b_first);
    dist_second = _mm_sub_epi16(a_second, b_second);
    dist_first  = _mm_mullo_epi16(dist_first, dist_first);
    dist_second = _mm_mullo_epi16(dist_second, dist_second);

    _mm_storeu_si128((__m128i *)dst, dist_first);
    _mm_storeu_si128((__m128i *)(dst + 8), dist_second);
}

static INLINE void read_dist_8(const uint16_t *dist, __m128i *dist_reg) {
    *dist_reg = _mm_loadu_si128((const __m128i *)dist);
}

static INLINE void read_dist_16(const uint16_t *dist, __m128i *reg_first, __m128i *reg_second) {
    read_dist_8(dist, reg_first);
    read_dist_8(dist + 8, reg_second);
}

// Average the value based on the number of values summed (9 for pixels away
// from the border, 4 for pixels in corners, and 6 for other edge values).
//
// Add in the rounding factor and shift, clamp to 16, invert and shift. Multiply
// by weight.
static __m128i average_8(__m128i sum, const __m128i *mul_constants, const int strength,
                         const int rounding, const int weight) {
    // _mm_srl_epi16 uses the lower 64 bit value for the shift.
    const __m128i strength_u128 = _mm_set_epi32(0, 0, 0, strength);
    const __m128i rounding_u16  = _mm_set1_epi16(rounding);
    const __m128i weight_u16    = _mm_set1_epi16(weight);
    const __m128i sixteen       = _mm_set1_epi16(16);

    // modifier * 3 / index;
    sum = _mm_mulhi_epu16(sum, *mul_constants);

    sum = _mm_adds_epu16(sum, rounding_u16);
    sum = _mm_srl_epi16(sum, strength_u128);

    // The maximum input to this comparison is UINT16_MAX * NEIGHBOR_CONSTANT_4
    // >> 16 (also NEIGHBOR_CONSTANT_4 -1) which is 49151 / 0xbfff / -16385
    // So this needs to use the epu16 version which did not come until SSE4.
    sum = _mm_min_epu16(sum, sixteen);

    sum = _mm_sub_epi16(sixteen, sum);

    return _mm_mullo_epi16(sum, weight_u16);
}

static __m128i average_4_4(__m128i sum, const __m128i *mul_constants, const int strength,
                           const int rounding, const int weight_0, const int weight_1) {
    // _mm_srl_epi16 uses the lower 64 bit value for the shift.
    const __m128i strength_u128 = _mm_set_epi32(0, 0, 0, strength);
    const __m128i rounding_u16  = _mm_set1_epi16(rounding);
    const __m128i weight_u16    = _mm_setr_epi16(
        weight_0, weight_0, weight_0, weight_0, weight_1, weight_1, weight_1, weight_1);
    const __m128i sixteen = _mm_set1_epi16(16);

    // modifier * 3 / index;
    sum = _mm_mulhi_epu16(sum, *mul_constants);

    sum = _mm_adds_epu16(sum, rounding_u16);
    sum = _mm_srl_epi16(sum, strength_u128);

    // The maximum input to this comparison is UINT16_MAX * NEIGHBOR_CONSTANT_4
    // >> 16 (also NEIGHBOR_CONSTANT_4 -1) which is 49151 / 0xbfff / -16385
    // So this needs to use the epu16 version which did not come until SSE4.
    sum = _mm_min_epu16(sum, sixteen);

    sum = _mm_sub_epi16(sixteen, sum);

    return _mm_mullo_epi16(sum, weight_u16);
}

static INLINE void average_16(__m128i *sum_0_u16, __m128i *sum_1_u16,
                              const __m128i *mul_constants_0, const __m128i *mul_constants_1,
                              const int strength, const int rounding, const int weight) {
    const __m128i strength_u128 = _mm_set_epi32(0, 0, 0, strength);
    const __m128i rounding_u16  = _mm_set1_epi16(rounding);
    const __m128i weight_u16    = _mm_set1_epi16(weight);
    const __m128i sixteen       = _mm_set1_epi16(16);
    __m128i       input_0, input_1;

    input_0 = _mm_mulhi_epu16(*sum_0_u16, *mul_constants_0);
    input_0 = _mm_adds_epu16(input_0, rounding_u16);

    input_1 = _mm_mulhi_epu16(*sum_1_u16, *mul_constants_1);
    input_1 = _mm_adds_epu16(input_1, rounding_u16);

    input_0 = _mm_srl_epi16(input_0, strength_u128);
    input_1 = _mm_srl_epi16(input_1, strength_u128);

    input_0 = _mm_min_epu16(input_0, sixteen);
    input_1 = _mm_min_epu16(input_1, sixteen);
    input_0 = _mm_sub_epi16(sixteen, input_0);
    input_1 = _mm_sub_epi16(sixteen, input_1);

    *sum_0_u16 = _mm_mullo_epi16(input_0, weight_u16);
    *sum_1_u16 = _mm_mullo_epi16(input_1, weight_u16);
}

// Add 'sum_u16' to 'count'. Multiply by 'pred' and add to 'accumulator.'
static void accumulate_and_store_8(const __m128i sum_u16, const uint8_t *pred, uint16_t *count,
                                   uint32_t *accumulator) {
    const __m128i pred_u8   = _mm_loadl_epi64((const __m128i *)pred);
    const __m128i zero      = _mm_setzero_si128();
    __m128i       count_u16 = _mm_loadu_si128((const __m128i *)count);
    __m128i       pred_u16  = _mm_cvtepu8_epi16(pred_u8);
    __m128i       pred_0_u32, pred_1_u32;
    __m128i       accum_0_u32, accum_1_u32;

    count_u16 = _mm_adds_epu16(count_u16, sum_u16);
    _mm_storeu_si128((__m128i *)count, count_u16);

    pred_u16 = _mm_mullo_epi16(sum_u16, pred_u16);

    pred_0_u32 = _mm_cvtepu16_epi32(pred_u16);
    pred_1_u32 = _mm_unpackhi_epi16(pred_u16, zero);

    accum_0_u32 = _mm_loadu_si128((const __m128i *)accumulator);
    accum_1_u32 = _mm_loadu_si128((const __m128i *)(accumulator + 4));

    accum_0_u32 = _mm_add_epi32(pred_0_u32, accum_0_u32);
    accum_1_u32 = _mm_add_epi32(pred_1_u32, accum_1_u32);

    _mm_storeu_si128((__m128i *)accumulator, accum_0_u32);
    _mm_storeu_si128((__m128i *)(accumulator + 4), accum_1_u32);
}

static INLINE void accumulate_and_store_16(const __m128i sum_0_u16, const __m128i sum_1_u16,
                                           const uint8_t *pred, uint16_t *count,
                                           uint32_t *accumulator) {
    const __m128i pred_u8     = _mm_loadu_si128((const __m128i *)pred);
    const __m128i zero        = _mm_setzero_si128();
    __m128i       count_0_u16 = _mm_loadu_si128((const __m128i *)count),
            count_1_u16       = _mm_loadu_si128((const __m128i *)(count + 8));
    __m128i pred_0_u16 = _mm_cvtepu8_epi16(pred_u8), pred_1_u16 = _mm_unpackhi_epi8(pred_u8, zero);
    __m128i pred_0_u32, pred_1_u32, pred_2_u32, pred_3_u32;
    __m128i accum_0_u32, accum_1_u32, accum_2_u32, accum_3_u32;

    count_0_u16 = _mm_adds_epu16(count_0_u16, sum_0_u16);
    _mm_storeu_si128((__m128i *)count, count_0_u16);

    count_1_u16 = _mm_adds_epu16(count_1_u16, sum_1_u16);
    _mm_storeu_si128((__m128i *)(count + 8), count_1_u16);

    pred_0_u16 = _mm_mullo_epi16(sum_0_u16, pred_0_u16);
    pred_1_u16 = _mm_mullo_epi16(sum_1_u16, pred_1_u16);

    pred_0_u32 = _mm_cvtepu16_epi32(pred_0_u16);
    pred_1_u32 = _mm_unpackhi_epi16(pred_0_u16, zero);
    pred_2_u32 = _mm_cvtepu16_epi32(pred_1_u16);
    pred_3_u32 = _mm_unpackhi_epi16(pred_1_u16, zero);

    accum_0_u32 = _mm_loadu_si128((const __m128i *)accumulator);
    accum_1_u32 = _mm_loadu_si128((const __m128i *)(accumulator + 4));
    accum_2_u32 = _mm_loadu_si128((const __m128i *)(accumulator + 8));
    accum_3_u32 = _mm_loadu_si128((const __m128i *)(accumulator + 12));

    accum_0_u32 = _mm_add_epi32(pred_0_u32, accum_0_u32);
    accum_1_u32 = _mm_add_epi32(pred_1_u32, accum_1_u32);
    accum_2_u32 = _mm_add_epi32(pred_2_u32, accum_2_u32);
    accum_3_u32 = _mm_add_epi32(pred_3_u32, accum_3_u32);

    _mm_storeu_si128((__m128i *)accumulator, accum_0_u32);
    _mm_storeu_si128((__m128i *)(accumulator + 4), accum_1_u32);
    _mm_storeu_si128((__m128i *)(accumulator + 8), accum_2_u32);
    _mm_storeu_si128((__m128i *)(accumulator + 12), accum_3_u32);
}

// Read in 8 pixels from y_dist. For each index i, compute y_dist[i-1] +
// y_dist[i] + y_dist[i+1] and store in sum as 16-bit unsigned int.
static INLINE void get_sum_8(const uint16_t *y_dist, __m128i *sum) {
    __m128i dist_reg, dist_left, dist_right;

    dist_reg   = _mm_loadu_si128((const __m128i *)y_dist);
    dist_left  = _mm_loadu_si128((const __m128i *)(y_dist - 1));
    dist_right = _mm_loadu_si128((const __m128i *)(y_dist + 1));

    *sum = _mm_adds_epu16(dist_reg, dist_left);
    *sum = _mm_adds_epu16(*sum, dist_right);
}

// Read in 16 pixels from y_dist. For each index i, compute y_dist[i-1] +
// y_dist[i] + y_dist[i+1]. Store the result for first 8 pixels in sum_first and
// the rest in sum_second.
static INLINE void get_sum_16(const uint16_t *y_dist, __m128i *sum_first, __m128i *sum_second) {
    get_sum_8(y_dist, sum_first);
    get_sum_8(y_dist + 8, sum_second);
}

// Read in a row of chroma values corresponds to a row of 16 luma values.
static INLINE void read_chroma_dist_row_16(int ss_x, const uint16_t *u_dist, const uint16_t *v_dist,
                                           __m128i *u_first, __m128i *u_second, __m128i *v_first,
                                           __m128i *v_second) {
    if (!ss_x) {
        // If there is no chroma subsampling in the horizontal direction, then we
        // need to load 16 entries from chroma.
        read_dist_16(u_dist, u_first, u_second);
        read_dist_16(v_dist, v_first, v_second);
    } else { // ss_x == 1
        // Otherwise, we only need to load 8 entries
        __m128i u_reg, v_reg;

        read_dist_8(u_dist, &u_reg);

        *u_first  = _mm_unpacklo_epi16(u_reg, u_reg);
        *u_second = _mm_unpackhi_epi16(u_reg, u_reg);

        read_dist_8(v_dist, &v_reg);

        *v_first  = _mm_unpacklo_epi16(v_reg, v_reg);
        *v_second = _mm_unpackhi_epi16(v_reg, v_reg);
    }
}

// Horizontal add unsigned 16-bit ints in src and store them as signed 32-bit
// int in dst.
static INLINE void hadd_epu16(__m128i *src, __m128i *dst) {
    const __m128i zero        = _mm_setzero_si128();
    const __m128i shift_right = _mm_srli_si128(*src, 2);

    const __m128i odd  = _mm_blend_epi16(shift_right, zero, 170);
    const __m128i even = _mm_blend_epi16(*src, zero, 170);

    *dst = _mm_add_epi32(even, odd);
}

// Add a row of luma distortion to 8 corresponding chroma mods.
static INLINE void add_luma_dist_to_8_chroma_mod(const uint16_t *y_dist, int ss_x, int ss_y,
                                                 __m128i *u_mod, __m128i *v_mod) {
    __m128i y_reg;
    if (!ss_x) {
        read_dist_8(y_dist, &y_reg);
        if (ss_y == 1) {
            __m128i y_tmp;
            read_dist_8(y_dist + DIST_STRIDE, &y_tmp);

            y_reg = _mm_adds_epu16(y_reg, y_tmp);
        }
    } else {
        __m128i y_first, y_second;
        read_dist_16(y_dist, &y_first, &y_second);
        if (ss_y == 1) {
            __m128i y_tmp_0, y_tmp_1;
            read_dist_16(y_dist + DIST_STRIDE, &y_tmp_0, &y_tmp_1);

            y_first  = _mm_adds_epu16(y_first, y_tmp_0);
            y_second = _mm_adds_epu16(y_second, y_tmp_1);
        }

        hadd_epu16(&y_first, &y_first);
        hadd_epu16(&y_second, &y_second);

        y_reg = _mm_packus_epi32(y_first, y_second);
    }

    *u_mod = _mm_adds_epu16(*u_mod, y_reg);
    *v_mod = _mm_adds_epu16(*v_mod, y_reg);
}

// Apply temporal filter to the luma components. This performs temporal
// filtering on a luma block of 16 X block_height. Use blk_fw as an array of
// size 4 for the weights for each of the 4 subblocks if blk_fw is not NULL,
// else use top_weight for top half, and bottom weight for bottom half.
static void av1_apply_temporal_filter_luma_16(
    const uint8_t *y_src, int y_src_stride, const uint8_t *y_pre, int y_pre_stride,
    const uint8_t *u_src, const uint8_t *v_src, int uv_src_stride, const uint8_t *u_pre,
    const uint8_t *v_pre, int uv_pre_stride, unsigned int block_width, unsigned int block_height,
    int ss_x, int ss_y, int strength, int use_whole_blk, uint32_t *y_accum, uint16_t *y_count,
    const uint16_t *y_dist, const uint16_t *u_dist, const uint16_t *v_dist,
    const int16_t *const *neighbors_first, const int16_t *const *neighbors_second, int top_weight,
    int bottom_weight, const int *blk_fw) {
    const int rounding = (1 << strength) >> 1;
    int       weight   = top_weight;

    __m128i mul_first, mul_second;

    __m128i sum_row_1_first, sum_row_1_second;
    __m128i sum_row_2_first, sum_row_2_second;
    __m128i sum_row_3_first, sum_row_3_second;

    __m128i u_first, u_second;
    __m128i v_first, v_second;

    __m128i sum_row_first;
    __m128i sum_row_second;

    // Loop variables
    unsigned int h;

    assert(strength >= 0);
    assert(strength <= 6);

    assert(block_width == 16);

    (void)block_width;

    // First row
    mul_first  = _mm_loadu_si128((const __m128i *)neighbors_first[0]);
    mul_second = _mm_loadu_si128((const __m128i *)neighbors_second[0]);

    // Add luma values
    get_sum_16(y_dist, &sum_row_2_first, &sum_row_2_second);
    get_sum_16(y_dist + DIST_STRIDE, &sum_row_3_first, &sum_row_3_second);

    sum_row_first  = _mm_adds_epu16(sum_row_2_first, sum_row_3_first);
    sum_row_second = _mm_adds_epu16(sum_row_2_second, sum_row_3_second);

    // Add chroma values
    read_chroma_dist_row_16(ss_x, u_dist, v_dist, &u_first, &u_second, &v_first, &v_second);

    sum_row_first  = _mm_adds_epu16(sum_row_first, u_first);
    sum_row_second = _mm_adds_epu16(sum_row_second, u_second);

    sum_row_first  = _mm_adds_epu16(sum_row_first, v_first);
    sum_row_second = _mm_adds_epu16(sum_row_second, v_second);

    // Get modifier and store result
    if (blk_fw) {
        sum_row_first  = average_8(sum_row_first, &mul_first, strength, rounding, blk_fw[0]);
        sum_row_second = average_8(sum_row_second, &mul_second, strength, rounding, blk_fw[1]);
    } else {
        average_16(
            &sum_row_first, &sum_row_second, &mul_first, &mul_second, strength, rounding, weight);
    }
    accumulate_and_store_16(sum_row_first, sum_row_second, y_pre, y_count, y_accum);

    y_src += y_src_stride;
    y_pre += y_pre_stride;
    y_count += y_pre_stride;
    y_accum += y_pre_stride;
    y_dist += DIST_STRIDE;

    u_src += uv_src_stride;
    u_pre += uv_pre_stride;
    u_dist += DIST_STRIDE;
    v_src += uv_src_stride;
    v_pre += uv_pre_stride;
    v_dist += DIST_STRIDE;

    // Then all the rows except the last one
    mul_first  = _mm_loadu_si128((const __m128i *)neighbors_first[1]);
    mul_second = _mm_loadu_si128((const __m128i *)neighbors_second[1]);

    for (h = 1; h < block_height - 1; ++h) {
        // Move the weight to bottom half
        if (!use_whole_blk && h == block_height / 2) {
            if (blk_fw) {
                blk_fw += 2;
            } else
                weight = bottom_weight;
        }
        // Shift the rows up
        sum_row_1_first  = sum_row_2_first;
        sum_row_1_second = sum_row_2_second;
        sum_row_2_first  = sum_row_3_first;
        sum_row_2_second = sum_row_3_second;

        // Add luma values to the modifier
        sum_row_first  = _mm_adds_epu16(sum_row_1_first, sum_row_2_first);
        sum_row_second = _mm_adds_epu16(sum_row_1_second, sum_row_2_second);

        get_sum_16(y_dist + DIST_STRIDE, &sum_row_3_first, &sum_row_3_second);

        sum_row_first  = _mm_adds_epu16(sum_row_first, sum_row_3_first);
        sum_row_second = _mm_adds_epu16(sum_row_second, sum_row_3_second);

        // Add chroma values to the modifier
        if (ss_y == 0 || h % 2 == 0) {
            // Only calculate the new chroma distortion if we are at a pixel that
            // corresponds to a new chroma row
            read_chroma_dist_row_16(ss_x, u_dist, v_dist, &u_first, &u_second, &v_first, &v_second);

            u_src += uv_src_stride;
            u_pre += uv_pre_stride;
            u_dist += DIST_STRIDE;
            v_src += uv_src_stride;
            v_pre += uv_pre_stride;
            v_dist += DIST_STRIDE;
        }

        sum_row_first  = _mm_adds_epu16(sum_row_first, u_first);
        sum_row_second = _mm_adds_epu16(sum_row_second, u_second);
        sum_row_first  = _mm_adds_epu16(sum_row_first, v_first);
        sum_row_second = _mm_adds_epu16(sum_row_second, v_second);

        // Get modifier and store result
        if (blk_fw) {
            sum_row_first  = average_8(sum_row_first, &mul_first, strength, rounding, blk_fw[0]);
            sum_row_second = average_8(sum_row_second, &mul_second, strength, rounding, blk_fw[1]);
        } else {
            average_16(&sum_row_first,
                       &sum_row_second,
                       &mul_first,
                       &mul_second,
                       strength,
                       rounding,
                       weight);
        }
        accumulate_and_store_16(sum_row_first, sum_row_second, y_pre, y_count, y_accum);

        y_src += y_src_stride;
        y_pre += y_pre_stride;
        y_count += y_pre_stride;
        y_accum += y_pre_stride;
        y_dist += DIST_STRIDE;
    }

    // The last row
    mul_first  = _mm_loadu_si128((const __m128i *)neighbors_first[0]);
    mul_second = _mm_loadu_si128((const __m128i *)neighbors_second[0]);

    // Shift the rows up
    sum_row_1_first  = sum_row_2_first;
    sum_row_1_second = sum_row_2_second;
    sum_row_2_first  = sum_row_3_first;
    sum_row_2_second = sum_row_3_second;

    // Add luma values to the modifier
    sum_row_first  = _mm_adds_epu16(sum_row_1_first, sum_row_2_first);
    sum_row_second = _mm_adds_epu16(sum_row_1_second, sum_row_2_second);

    // Add chroma values to the modifier
    if (ss_y == 0) {
        // Only calculate the new chroma distortion if we are at a pixel that
        // corresponds to a new chroma row
        read_chroma_dist_row_16(ss_x, u_dist, v_dist, &u_first, &u_second, &v_first, &v_second);
    }

    sum_row_first  = _mm_adds_epu16(sum_row_first, u_first);
    sum_row_second = _mm_adds_epu16(sum_row_second, u_second);
    sum_row_first  = _mm_adds_epu16(sum_row_first, v_first);
    sum_row_second = _mm_adds_epu16(sum_row_second, v_second);

    // Get modifier and store result
    if (blk_fw) {
        sum_row_first  = average_8(sum_row_first, &mul_first, strength, rounding, blk_fw[0]);
        sum_row_second = average_8(sum_row_second, &mul_second, strength, rounding, blk_fw[1]);
    } else {
        average_16(
            &sum_row_first, &sum_row_second, &mul_first, &mul_second, strength, rounding, weight);
    }
    accumulate_and_store_16(sum_row_first, sum_row_second, y_pre, y_count, y_accum);
}

// Perform temporal filter for the luma component.
static void av1_apply_temporal_filter_luma(
    const uint8_t *y_src, int y_src_stride, const uint8_t *y_pre, int y_pre_stride,
    const uint8_t *u_src, const uint8_t *v_src, int uv_src_stride, const uint8_t *u_pre,
    const uint8_t *v_pre, int uv_pre_stride, unsigned int block_width, unsigned int block_height,
    int ss_x, int ss_y, int strength, const int *blk_fw, int use_whole_blk, uint32_t *y_accum,
    uint16_t *y_count, const uint16_t *y_dist, const uint16_t *u_dist, const uint16_t *v_dist) {
    unsigned int       blk_col = 0, uv_blk_col = 0;
    const unsigned int blk_col_step = 16, uv_blk_col_step = 16 >> ss_x;
    const unsigned int mid_width = block_width >> 1, last_width = block_width - blk_col_step;
    int top_weight = blk_fw[0], bottom_weight = use_whole_blk ? blk_fw[0] : blk_fw[2];
    const int16_t *const *neighbors_first;
    const int16_t *const *neighbors_second;

    if (block_width == 16) {
        // Special Case: The blockwidth is 16 and we are operating on a row of 16
        // chroma pixels. In this case, we can't use the usualy left-midle-right
        // pattern. We also don't support splitting now.
        neighbors_first  = luma_left_column_neighbors;
        neighbors_second = luma_right_column_neighbors;
        if (use_whole_blk) {
            av1_apply_temporal_filter_luma_16(y_src + blk_col,
                                              y_src_stride,
                                              y_pre + blk_col,
                                              y_pre_stride,
                                              u_src + uv_blk_col,
                                              v_src + uv_blk_col,
                                              uv_src_stride,
                                              u_pre + uv_blk_col,
                                              v_pre + uv_blk_col,
                                              uv_pre_stride,
                                              16,
                                              block_height,
                                              ss_x,
                                              ss_y,
                                              strength,
                                              use_whole_blk,
                                              y_accum + blk_col,
                                              y_count + blk_col,
                                              y_dist + blk_col,
                                              u_dist + uv_blk_col,
                                              v_dist + uv_blk_col,
                                              neighbors_first,
                                              neighbors_second,
                                              top_weight,
                                              bottom_weight,
                                              NULL);
        } else {
            av1_apply_temporal_filter_luma_16(y_src + blk_col,
                                              y_src_stride,
                                              y_pre + blk_col,
                                              y_pre_stride,
                                              u_src + uv_blk_col,
                                              v_src + uv_blk_col,
                                              uv_src_stride,
                                              u_pre + uv_blk_col,
                                              v_pre + uv_blk_col,
                                              uv_pre_stride,
                                              16,
                                              block_height,
                                              ss_x,
                                              ss_y,
                                              strength,
                                              use_whole_blk,
                                              y_accum + blk_col,
                                              y_count + blk_col,
                                              y_dist + blk_col,
                                              u_dist + uv_blk_col,
                                              v_dist + uv_blk_col,
                                              neighbors_first,
                                              neighbors_second,
                                              0,
                                              0,
                                              blk_fw);
        }

        return;
    }

    // Left
    neighbors_first  = luma_left_column_neighbors;
    neighbors_second = luma_middle_column_neighbors;
    av1_apply_temporal_filter_luma_16(y_src + blk_col,
                                      y_src_stride,
                                      y_pre + blk_col,
                                      y_pre_stride,
                                      u_src + uv_blk_col,
                                      v_src + uv_blk_col,
                                      uv_src_stride,
                                      u_pre + uv_blk_col,
                                      v_pre + uv_blk_col,
                                      uv_pre_stride,
                                      16,
                                      block_height,
                                      ss_x,
                                      ss_y,
                                      strength,
                                      use_whole_blk,
                                      y_accum + blk_col,
                                      y_count + blk_col,
                                      y_dist + blk_col,
                                      u_dist + uv_blk_col,
                                      v_dist + uv_blk_col,
                                      neighbors_first,
                                      neighbors_second,
                                      top_weight,
                                      bottom_weight,
                                      NULL);

    blk_col += blk_col_step;
    uv_blk_col += uv_blk_col_step;

    // Middle First
    neighbors_first = luma_middle_column_neighbors;
    for (; blk_col < mid_width; blk_col += blk_col_step, uv_blk_col += uv_blk_col_step) {
        av1_apply_temporal_filter_luma_16(y_src + blk_col,
                                          y_src_stride,
                                          y_pre + blk_col,
                                          y_pre_stride,
                                          u_src + uv_blk_col,
                                          v_src + uv_blk_col,
                                          uv_src_stride,
                                          u_pre + uv_blk_col,
                                          v_pre + uv_blk_col,
                                          uv_pre_stride,
                                          16,
                                          block_height,
                                          ss_x,
                                          ss_y,
                                          strength,
                                          use_whole_blk,
                                          y_accum + blk_col,
                                          y_count + blk_col,
                                          y_dist + blk_col,
                                          u_dist + uv_blk_col,
                                          v_dist + uv_blk_col,
                                          neighbors_first,
                                          neighbors_second,
                                          top_weight,
                                          bottom_weight,
                                          NULL);
    }

    if (!use_whole_blk) {
        top_weight    = blk_fw[1];
        bottom_weight = blk_fw[3];
    }

    // Middle Second
    for (; blk_col < last_width; blk_col += blk_col_step, uv_blk_col += uv_blk_col_step) {
        av1_apply_temporal_filter_luma_16(y_src + blk_col,
                                          y_src_stride,
                                          y_pre + blk_col,
                                          y_pre_stride,
                                          u_src + uv_blk_col,
                                          v_src + uv_blk_col,
                                          uv_src_stride,
                                          u_pre + uv_blk_col,
                                          v_pre + uv_blk_col,
                                          uv_pre_stride,
                                          16,
                                          block_height,
                                          ss_x,
                                          ss_y,
                                          strength,
                                          use_whole_blk,
                                          y_accum + blk_col,
                                          y_count + blk_col,
                                          y_dist + blk_col,
                                          u_dist + uv_blk_col,
                                          v_dist + uv_blk_col,
                                          neighbors_first,
                                          neighbors_second,
                                          top_weight,
                                          bottom_weight,
                                          NULL);
    }

    // Right
    neighbors_second = luma_right_column_neighbors;
    av1_apply_temporal_filter_luma_16(y_src + blk_col,
                                      y_src_stride,
                                      y_pre + blk_col,
                                      y_pre_stride,
                                      u_src + uv_blk_col,
                                      v_src + uv_blk_col,
                                      uv_src_stride,
                                      u_pre + uv_blk_col,
                                      v_pre + uv_blk_col,
                                      uv_pre_stride,
                                      16,
                                      block_height,
                                      ss_x,
                                      ss_y,
                                      strength,
                                      use_whole_blk,
                                      y_accum + blk_col,
                                      y_count + blk_col,
                                      y_dist + blk_col,
                                      u_dist + uv_blk_col,
                                      v_dist + uv_blk_col,
                                      neighbors_first,
                                      neighbors_second,
                                      top_weight,
                                      bottom_weight,
                                      NULL);
}

// Apply temporal filter to the chroma components. This performs temporal
// filtering on a chroma block of 8 X uv_height. If blk_fw is not NULL, use
// blk_fw as an array of size 4 for the weights for each of the 4 subblocks,
// else use top_weight for top half, and bottom weight for bottom half.
static void av1_apply_temporal_filter_chroma_8(
    const uint8_t *y_src, int y_src_stride, const uint8_t *y_pre, int y_pre_stride,
    const uint8_t *u_src, const uint8_t *v_src, int uv_src_stride, const uint8_t *u_pre,
    const uint8_t *v_pre, int uv_pre_stride, unsigned int uv_block_width,
    unsigned int uv_block_height, int ss_x, int ss_y, int strength, uint32_t *u_accum,
    uint16_t *u_count, uint32_t *v_accum, uint16_t *v_count, const uint16_t *y_dist,
    const uint16_t *u_dist, const uint16_t *v_dist, const int16_t *const *neighbors, int top_weight,
    int bottom_weight, const int *blk_fw) {
    const int rounding = (1 << strength) >> 1;
    int       weight   = top_weight;

    __m128i mul;

    __m128i u_sum_row_1, u_sum_row_2, u_sum_row_3;
    __m128i v_sum_row_1, v_sum_row_2, v_sum_row_3;

    __m128i u_sum_row, v_sum_row;

    // Loop variable
    unsigned int h;

    (void)uv_block_width;

    // First row
    mul = _mm_loadu_si128((const __m128i *)neighbors[0]);

    // Add chroma values
    get_sum_8(u_dist, &u_sum_row_2);
    get_sum_8(u_dist + DIST_STRIDE, &u_sum_row_3);

    u_sum_row = _mm_adds_epu16(u_sum_row_2, u_sum_row_3);

    get_sum_8(v_dist, &v_sum_row_2);
    get_sum_8(v_dist + DIST_STRIDE, &v_sum_row_3);

    v_sum_row = _mm_adds_epu16(v_sum_row_2, v_sum_row_3);

    // Add luma values
    add_luma_dist_to_8_chroma_mod(y_dist, ss_x, ss_y, &u_sum_row, &v_sum_row);

    // Get modifier and store result
    if (blk_fw) {
        u_sum_row = average_4_4(u_sum_row, &mul, strength, rounding, blk_fw[0], blk_fw[1]);
        v_sum_row = average_4_4(v_sum_row, &mul, strength, rounding, blk_fw[0], blk_fw[1]);
    } else {
        u_sum_row = average_8(u_sum_row, &mul, strength, rounding, weight);
        v_sum_row = average_8(v_sum_row, &mul, strength, rounding, weight);
    }
    accumulate_and_store_8(u_sum_row, u_pre, u_count, u_accum);
    accumulate_and_store_8(v_sum_row, v_pre, v_count, v_accum);

    u_src += uv_src_stride;
    u_pre += uv_pre_stride;
    u_dist += DIST_STRIDE;
    v_src += uv_src_stride;
    v_pre += uv_pre_stride;
    v_dist += DIST_STRIDE;
    u_count += uv_pre_stride;
    u_accum += uv_pre_stride;
    v_count += uv_pre_stride;
    v_accum += uv_pre_stride;

    y_src += y_src_stride * (1 + ss_y);
    y_pre += y_pre_stride * (1 + ss_y);
    y_dist += DIST_STRIDE * (1 + ss_y);

    // Then all the rows except the last one
    mul = _mm_loadu_si128((const __m128i *)neighbors[1]);

    for (h = 1; h < uv_block_height - 1; ++h) {
        // Move the weight pointer to the bottom half of the blocks
        if (h == uv_block_height / 2) {
            if (blk_fw) {
                blk_fw += 2;
            } else
                weight = bottom_weight;
        }

        // Shift the rows up
        u_sum_row_1 = u_sum_row_2;
        u_sum_row_2 = u_sum_row_3;

        v_sum_row_1 = v_sum_row_2;
        v_sum_row_2 = v_sum_row_3;

        // Add chroma values
        u_sum_row = _mm_adds_epu16(u_sum_row_1, u_sum_row_2);
        get_sum_8(u_dist + DIST_STRIDE, &u_sum_row_3);
        u_sum_row = _mm_adds_epu16(u_sum_row, u_sum_row_3);

        v_sum_row = _mm_adds_epu16(v_sum_row_1, v_sum_row_2);
        get_sum_8(v_dist + DIST_STRIDE, &v_sum_row_3);
        v_sum_row = _mm_adds_epu16(v_sum_row, v_sum_row_3);

        // Add luma values
        add_luma_dist_to_8_chroma_mod(y_dist, ss_x, ss_y, &u_sum_row, &v_sum_row);

        // Get modifier and store result
        if (blk_fw) {
            u_sum_row = average_4_4(u_sum_row, &mul, strength, rounding, blk_fw[0], blk_fw[1]);
            v_sum_row = average_4_4(v_sum_row, &mul, strength, rounding, blk_fw[0], blk_fw[1]);
        } else {
            u_sum_row = average_8(u_sum_row, &mul, strength, rounding, weight);
            v_sum_row = average_8(v_sum_row, &mul, strength, rounding, weight);
        }

        accumulate_and_store_8(u_sum_row, u_pre, u_count, u_accum);
        accumulate_and_store_8(v_sum_row, v_pre, v_count, v_accum);

        u_src += uv_src_stride;
        u_pre += uv_pre_stride;
        u_dist += DIST_STRIDE;
        v_src += uv_src_stride;
        v_pre += uv_pre_stride;
        v_dist += DIST_STRIDE;
        u_count += uv_pre_stride;
        u_accum += uv_pre_stride;
        v_count += uv_pre_stride;
        v_accum += uv_pre_stride;

        y_src += y_src_stride * (1 + ss_y);
        y_pre += y_pre_stride * (1 + ss_y);
        y_dist += DIST_STRIDE * (1 + ss_y);
    }

    // The last row
    mul = _mm_loadu_si128((const __m128i *)neighbors[0]);

    // Shift the rows up
    u_sum_row_1 = u_sum_row_2;
    u_sum_row_2 = u_sum_row_3;

    v_sum_row_1 = v_sum_row_2;
    v_sum_row_2 = v_sum_row_3;

    // Add chroma values
    u_sum_row = _mm_adds_epu16(u_sum_row_1, u_sum_row_2);
    v_sum_row = _mm_adds_epu16(v_sum_row_1, v_sum_row_2);

    // Add luma values
    add_luma_dist_to_8_chroma_mod(y_dist, ss_x, ss_y, &u_sum_row, &v_sum_row);

    // Get modifier and store result
    if (blk_fw) {
        u_sum_row = average_4_4(u_sum_row, &mul, strength, rounding, blk_fw[0], blk_fw[1]);
        v_sum_row = average_4_4(v_sum_row, &mul, strength, rounding, blk_fw[0], blk_fw[1]);
    } else {
        u_sum_row = average_8(u_sum_row, &mul, strength, rounding, weight);
        v_sum_row = average_8(v_sum_row, &mul, strength, rounding, weight);
    }

    accumulate_and_store_8(u_sum_row, u_pre, u_count, u_accum);
    accumulate_and_store_8(v_sum_row, v_pre, v_count, v_accum);
}

// Perform temporal filter for the chroma components.
static void av1_apply_temporal_filter_chroma(
    const uint8_t *y_src, int y_src_stride, const uint8_t *y_pre, int y_pre_stride,
    const uint8_t *u_src, const uint8_t *v_src, int uv_src_stride, const uint8_t *u_pre,
    const uint8_t *v_pre, int uv_pre_stride, unsigned int block_width, unsigned int block_height,
    int ss_x, int ss_y, int strength, const int *blk_fw, int use_whole_blk, uint32_t *u_accum,
    uint16_t *u_count, uint32_t *v_accum, uint16_t *v_count, const uint16_t *y_dist,
    const uint16_t *u_dist, const uint16_t *v_dist) {
    const unsigned int uv_width = block_width >> ss_x, uv_height = block_height >> ss_y;

    unsigned int       blk_col = 0, uv_blk_col = 0;
    const unsigned int uv_blk_col_step = 8, blk_col_step = 8 << ss_x;
    const unsigned int uv_mid_width = uv_width >> 1, uv_last_width = uv_width - uv_blk_col_step;
    int top_weight = blk_fw[0], bottom_weight = use_whole_blk ? blk_fw[0] : blk_fw[2];
    const int16_t *const *neighbors;

    if (uv_width == 8) {
        // Special Case: We are subsampling in x direction on a 16x16 block. Since
        // we are operating on a row of 8 chroma pixels, we can't use the usual
        // left-middle-right pattern.
        assert(ss_x);

        if (ss_y) {
            neighbors = chroma_double_ss_single_column_neighbors;
        } else
            neighbors = chroma_single_ss_single_column_neighbors;
        if (use_whole_blk) {
            av1_apply_temporal_filter_chroma_8(y_src + blk_col,
                                               y_src_stride,
                                               y_pre + blk_col,
                                               y_pre_stride,
                                               u_src + uv_blk_col,
                                               v_src + uv_blk_col,
                                               uv_src_stride,
                                               u_pre + uv_blk_col,
                                               v_pre + uv_blk_col,
                                               uv_pre_stride,
                                               uv_width,
                                               uv_height,
                                               ss_x,
                                               ss_y,
                                               strength,
                                               u_accum + uv_blk_col,
                                               u_count + uv_blk_col,
                                               v_accum + uv_blk_col,
                                               v_count + uv_blk_col,
                                               y_dist + blk_col,
                                               u_dist + uv_blk_col,
                                               v_dist + uv_blk_col,
                                               neighbors,
                                               top_weight,
                                               bottom_weight,
                                               NULL);
        } else {
            av1_apply_temporal_filter_chroma_8(y_src + blk_col,
                                               y_src_stride,
                                               y_pre + blk_col,
                                               y_pre_stride,
                                               u_src + uv_blk_col,
                                               v_src + uv_blk_col,
                                               uv_src_stride,
                                               u_pre + uv_blk_col,
                                               v_pre + uv_blk_col,
                                               uv_pre_stride,
                                               uv_width,
                                               uv_height,
                                               ss_x,
                                               ss_y,
                                               strength,
                                               u_accum + uv_blk_col,
                                               u_count + uv_blk_col,
                                               v_accum + uv_blk_col,
                                               v_count + uv_blk_col,
                                               y_dist + blk_col,
                                               u_dist + uv_blk_col,
                                               v_dist + uv_blk_col,
                                               neighbors,
                                               0,
                                               0,
                                               blk_fw);
        }

        return;
    }

    // Left
    if (ss_x && ss_y) {
        neighbors = chroma_double_ss_left_column_neighbors;
    } else if (ss_x || ss_y) {
        neighbors = chroma_single_ss_left_column_neighbors;
    } else
        neighbors = chroma_no_ss_left_column_neighbors;
    av1_apply_temporal_filter_chroma_8(y_src + blk_col,
                                       y_src_stride,
                                       y_pre + blk_col,
                                       y_pre_stride,
                                       u_src + uv_blk_col,
                                       v_src + uv_blk_col,
                                       uv_src_stride,
                                       u_pre + uv_blk_col,
                                       v_pre + uv_blk_col,
                                       uv_pre_stride,
                                       uv_width,
                                       uv_height,
                                       ss_x,
                                       ss_y,
                                       strength,
                                       u_accum + uv_blk_col,
                                       u_count + uv_blk_col,
                                       v_accum + uv_blk_col,
                                       v_count + uv_blk_col,
                                       y_dist + blk_col,
                                       u_dist + uv_blk_col,
                                       v_dist + uv_blk_col,
                                       neighbors,
                                       top_weight,
                                       bottom_weight,
                                       NULL);

    blk_col += blk_col_step;
    uv_blk_col += uv_blk_col_step;

    // Middle First
    if (ss_x && ss_y) {
        neighbors = chroma_double_ss_middle_column_neighbors;
    } else if (ss_x || ss_y) {
        neighbors = chroma_single_ss_middle_column_neighbors;
    } else
        neighbors = chroma_no_ss_middle_column_neighbors;
    for (; uv_blk_col < uv_mid_width; blk_col += blk_col_step, uv_blk_col += uv_blk_col_step) {
        av1_apply_temporal_filter_chroma_8(y_src + blk_col,
                                           y_src_stride,
                                           y_pre + blk_col,
                                           y_pre_stride,
                                           u_src + uv_blk_col,
                                           v_src + uv_blk_col,
                                           uv_src_stride,
                                           u_pre + uv_blk_col,
                                           v_pre + uv_blk_col,
                                           uv_pre_stride,
                                           uv_width,
                                           uv_height,
                                           ss_x,
                                           ss_y,
                                           strength,
                                           u_accum + uv_blk_col,
                                           u_count + uv_blk_col,
                                           v_accum + uv_blk_col,
                                           v_count + uv_blk_col,
                                           y_dist + blk_col,
                                           u_dist + uv_blk_col,
                                           v_dist + uv_blk_col,
                                           neighbors,
                                           top_weight,
                                           bottom_weight,
                                           NULL);
    }

    if (!use_whole_blk) {
        top_weight    = blk_fw[1];
        bottom_weight = blk_fw[3];
    }

    // Middle Second
    for (; uv_blk_col < uv_last_width; blk_col += blk_col_step, uv_blk_col += uv_blk_col_step) {
        av1_apply_temporal_filter_chroma_8(y_src + blk_col,
                                           y_src_stride,
                                           y_pre + blk_col,
                                           y_pre_stride,
                                           u_src + uv_blk_col,
                                           v_src + uv_blk_col,
                                           uv_src_stride,
                                           u_pre + uv_blk_col,
                                           v_pre + uv_blk_col,
                                           uv_pre_stride,
                                           uv_width,
                                           uv_height,
                                           ss_x,
                                           ss_y,
                                           strength,
                                           u_accum + uv_blk_col,
                                           u_count + uv_blk_col,
                                           v_accum + uv_blk_col,
                                           v_count + uv_blk_col,
                                           y_dist + blk_col,
                                           u_dist + uv_blk_col,
                                           v_dist + uv_blk_col,
                                           neighbors,
                                           top_weight,
                                           bottom_weight,
                                           NULL);
    }

    // Right
    if (ss_x && ss_y) {
        neighbors = chroma_double_ss_right_column_neighbors;
    } else if (ss_x || ss_y) {
        neighbors = chroma_single_ss_right_column_neighbors;
    } else
        neighbors = chroma_no_ss_right_column_neighbors;
    av1_apply_temporal_filter_chroma_8(y_src + blk_col,
                                       y_src_stride,
                                       y_pre + blk_col,
                                       y_pre_stride,
                                       u_src + uv_blk_col,
                                       v_src + uv_blk_col,
                                       uv_src_stride,
                                       u_pre + uv_blk_col,
                                       v_pre + uv_blk_col,
                                       uv_pre_stride,
                                       uv_width,
                                       uv_height,
                                       ss_x,
                                       ss_y,
                                       strength,
                                       u_accum + uv_blk_col,
                                       u_count + uv_blk_col,
                                       v_accum + uv_blk_col,
                                       v_count + uv_blk_col,
                                       y_dist + blk_col,
                                       u_dist + uv_blk_col,
                                       v_dist + uv_blk_col,
                                       neighbors,
                                       top_weight,
                                       bottom_weight,
                                       NULL);
}

void svt_av1_apply_temporal_filter_sse4_1(
    const uint8_t *y_src, int y_src_stride, const uint8_t *y_pre, int y_pre_stride,
    const uint8_t *u_src, const uint8_t *v_src, int uv_src_stride, const uint8_t *u_pre,
    const uint8_t *v_pre, int uv_pre_stride, unsigned int block_width, unsigned int block_height,
    int ss_x, int ss_y, int strength, const int *blk_fw, int use_whole_blk, uint32_t *y_accum,
    uint16_t *y_count, uint32_t *u_accum, uint16_t *u_count, uint32_t *v_accum, uint16_t *v_count) {
    const unsigned int chroma_height = block_height >> ss_y, chroma_width = block_width >> ss_x;

    DECLARE_ALIGNED(16, uint16_t, y_dist[BH * DIST_STRIDE]) = {0};
    DECLARE_ALIGNED(16, uint16_t, u_dist[BH * DIST_STRIDE]) = {0};
    DECLARE_ALIGNED(16, uint16_t, v_dist[BH * DIST_STRIDE]) = {0};
    const int *blk_fw_ptr                                   = blk_fw;

    uint16_t *     y_dist_ptr = y_dist + 1, *u_dist_ptr = u_dist + 1, *v_dist_ptr = v_dist + 1;
    const uint8_t *y_src_ptr = y_src, *u_src_ptr = u_src, *v_src_ptr = v_src;
    const uint8_t *y_pre_ptr = y_pre, *u_pre_ptr = u_pre, *v_pre_ptr = v_pre;

    // Loop variables
    unsigned int row, blk_col;

    assert(block_width <= BW && "block width too large");
    assert(block_height <= BH && "block height too large");
    assert(block_width % 16 == 0 && "block width must be multiple of 16");
    assert(block_height % 2 == 0 && "block height must be even");
    assert((ss_x == 0 || ss_x == 1) && (ss_y == 0 || ss_y == 1) && "invalid chroma subsampling");
    assert(strength >= 0 && strength <= 6 && "invalid temporal filter strength");
    assert(blk_fw[0] >= 0 && "filter weight must be positive");
    assert((use_whole_blk || (blk_fw[1] >= 0 && blk_fw[2] >= 0 && blk_fw[3] >= 0)) &&
           "subblock filter weight must be positive");
    assert(blk_fw[0] <= 2 && "sublock filter weight must be less than 2");
    assert((use_whole_blk || (blk_fw[1] <= 2 && blk_fw[2] <= 2 && blk_fw[3] <= 2)) &&
           "subblock filter weight must be less than 2");

    // Precompute the difference sqaured
    for (row = 0; row < block_height; row++) {
        for (blk_col = 0; blk_col < block_width; blk_col += 16) {
            store_dist_16(y_src_ptr + blk_col, y_pre_ptr + blk_col, y_dist_ptr + blk_col);
        }
        y_src_ptr += y_src_stride;
        y_pre_ptr += y_pre_stride;
        y_dist_ptr += DIST_STRIDE;
    }

    for (row = 0; row < chroma_height; row++) {
        for (blk_col = 0; blk_col < chroma_width; blk_col += 8) {
            store_dist_8(u_src_ptr + blk_col, u_pre_ptr + blk_col, u_dist_ptr + blk_col);
            store_dist_8(v_src_ptr + blk_col, v_pre_ptr + blk_col, v_dist_ptr + blk_col);
        }

        u_src_ptr += uv_src_stride;
        u_pre_ptr += uv_pre_stride;
        u_dist_ptr += DIST_STRIDE;
        v_src_ptr += uv_src_stride;
        v_pre_ptr += uv_pre_stride;
        v_dist_ptr += DIST_STRIDE;
    }

    y_dist_ptr = y_dist + 1;
    u_dist_ptr = u_dist + 1;
    v_dist_ptr = v_dist + 1;

    av1_apply_temporal_filter_luma(y_src,
                                   y_src_stride,
                                   y_pre,
                                   y_pre_stride,
                                   u_src,
                                   v_src,
                                   uv_src_stride,
                                   u_pre,
                                   v_pre,
                                   uv_pre_stride,
                                   block_width,
                                   block_height,
                                   ss_x,
                                   ss_y,
                                   strength,
                                   blk_fw_ptr,
                                   use_whole_blk,
                                   y_accum,
                                   y_count,
                                   y_dist_ptr,
                                   u_dist_ptr,
                                   v_dist_ptr);

    av1_apply_temporal_filter_chroma(y_src,
                                     y_src_stride,
                                     y_pre,
                                     y_pre_stride,
                                     u_src,
                                     v_src,
                                     uv_src_stride,
                                     u_pre,
                                     v_pre,
                                     uv_pre_stride,
                                     block_width,
                                     block_height,
                                     ss_x,
                                     ss_y,
                                     strength,
                                     blk_fw_ptr,
                                     use_whole_blk,
                                     u_accum,
                                     u_count,
                                     v_accum,
                                     v_count,
                                     y_dist_ptr,
                                     u_dist_ptr,
                                     v_dist_ptr);
}
