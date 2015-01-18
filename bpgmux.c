#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <limits.h>
#include <getopt.h>
#include <math.h>
#include <unistd.h>

#include "bpgfmt.h"
#include "libbpg.h"

#define IMAGE_HEADER_MAGIC 0x425047fb

#define DEFAULT_OUTFILENAME "out.bpg"
#define DEFAULT_COLORSPACE BPG_CS_YCbCr_BT709

#define BIT_DEPTH_MAX 14

enum NALType {
    TRAIL_N    = 0,
    TRAIL_R    = 1,
    TSA_N      = 2,
    TSA_R      = 3,
    STSA_N     = 4,
    STSA_R     = 5,
    RADL_N     = 6,
    RADL_R     = 7,
    RASL_N     = 8,
    RASL_R     = 9,
    BLA_W_LP   = 16,
    BLA_W_RADL = 17,
    BLA_N_LP   = 18,
    IDR_W_RADL = 19,
    IDR_N_LP   = 20,
    CRA_NUT    = 21,
    RSV_IRAP   = 23,
    VPS        = 32,
    SPS        = 33,
    PPS        = 34,
    AUD        = 35,
    EOS        = 36,
    EOB        = 37,
    FD         = 38,
    PREFIX_SEI = 39,
    SUFFIX_SEI = 40,
};

typedef enum {
    PARALLEL_NONE  = 0,
    PARALLEL_TILES = 1,
    PARALLEL_WPP   = 2,

    PARALLEL_BOTH  = 3,
} HEVCParallelismEnum;

typedef enum {
	INTER_NONE   = 0,
    INTER_AMP    = 1,
    INTER_TMVP   = 2,
    INTER_TU_RQT = 4,

    INTER_ALL    = 7,
} HEVCInterToolsEnum;

typedef struct HEVCVideoConfig {
    uint32_t width;
    uint32_t height;
    uint32_t min_cb_size;
    uint32_t fps_num;
    uint32_t fps_den;

    uint8_t bit_depth;
    uint8_t limited_range;
    BPGImageFormatEnum pixel_format;
    BPGColorSpaceEnum color_space;

    HEVCInterToolsEnum inter_tools;
    HEVCParallelismEnum parallel;
    uint8_t dependent_slices;

    uint8_t *msps;
    uint32_t msps_len;
    uint32_t pps_idx;
} HEVCVideoConfig;

typedef struct BPGMuxerContext {
    BPGMetaData *first_md;

    HEVCVideoConfig color;
    HEVCVideoConfig alpha;
    uint8_t *color_buf;
    uint8_t *alpha_buf;

    uint16_t loop_count; /* animations: number of loops. 0=infinite */
    /* animations: the frame delay is a multiple of
       frame_delay_num/frame_delay_den seconds */
    uint16_t frame_delay_num;
    uint16_t frame_delay_den;

    uint16_t frame_count;
    uint16_t frame_ticks;
    uint16_t *frame_duration_tab;
    uint16_t frame_duration_tab_size;

    uint16_t frame_limit;

    BPGColorSpaceEnum color_space;
    uint8_t limited_range;
    uint8_t premultiplied_alpha;
    uint8_t cmyk;
} BPGMuxerContext;

/* return the position of the end of the NAL or -1 if error */
static int extract_nal(uint8_t **pnal_buf, uint32_t *pnal_len,
                       const uint8_t *buf, const int buf_len)
{
    int idx, start, end, len;
    uint8_t *nal_buf;
    uint32_t nal_len;

    end = find_nal_end(buf, buf_len);
    if (end < 0)
        return -1;
    if (buf[2] == 1)
        start = 3;
    else
        start = 4;
    len = end - start;

    nal_buf = malloc(len);
    nal_len = 0;
    idx = start;
    while (idx < end) {
        if (idx + 2 < end && buf[idx] == 0 && buf[idx + 1] == 0 && buf[idx + 2] == 3) {
            nal_buf[nal_len++] = 0;
            nal_buf[nal_len++] = 0;
            idx += 3;
        } else {
            nal_buf[nal_len++] = buf[idx++];
        }
    }
    while (idx < end) {
        nal_buf[nal_len++] = buf[idx++];
    }
    *pnal_buf = nal_buf;
    *pnal_len = nal_len;
    return idx;
}

/* check each visual frames to ensure BPG-valid parameters */
static int parse_slice(const uint8_t *buf, const int len, const int nut,
                       const HEVCVideoConfig *cfg, const HEVCInterToolsEnum inter_tools,
                       uint8_t *last_poc)
{
    uint8_t is_IRAP, first_slice_segment_in_pic;
    uint8_t slice_pic_order_cnt_lsb;
    GetBitState gb_s, *gb = &gb_s;

    if (nut <= TRAIL_R && (buf[2] & 0x80)) {
        is_IRAP = 0;
    } else if (nut >= BLA_W_LP && nut <= RSV_IRAP && (buf[2] & 0x80)) {
        is_IRAP = 1;
    } else if (nut == PPS) {
        return 0;
    } else {
        fprintf(stderr, "unexpected NAL type %d\n", nut);
        return -1;
    }

    init_get_bits(gb, buf, len);
    skip_bits(gb, 16);  // nal header

    first_slice_segment_in_pic = (uint8_t) get_bits(gb, 1);
    if (is_IRAP) {
        skip_bits(gb, 1);   /* no_output_of_prior_pics_flag */
    }
    if (get_ue_golomb(gb) != 0) {
        fprintf(stderr, "slice_pic_parameter_set_id should be 0\n");
        return -1;
    }

    if (!first_slice_segment_in_pic) {
        uint32_t slice_segment_addr_len;
        uint32_t w1, h1;

        if (cfg->dependent_slices)
            if (get_bits(gb, 1))
                return 0;

        /* add back padding to get to (multiple of min_cb_size) */
        w1 = (cfg->width  + cfg->min_cb_size - 1) & ~(cfg->min_cb_size - 1);
        h1 = (cfg->height + cfg->min_cb_size - 1) & ~(cfg->min_cb_size - 1);
        slice_segment_addr_len = lrint(ceil(log(w1 * h1) / log(2)));

        gb->idx += slice_segment_addr_len;  /* slice_segment_address */
    }

    slice_pic_order_cnt_lsb = *last_poc;
    if (!cfg->dependent_slices) {
        uint32_t i, slice_type;

        const int num_extra_slice_header_bits = 0;
        for (i = 0; i < num_extra_slice_header_bits; i++)
            skip_bits(gb, 1);   /* slice_reserved_flag[ i ] */

        slice_type = get_ue_golomb(gb);
        if (slice_type != 2 && is_IRAP) {
            fprintf(stderr, "slice_type (%d) supposed to be IRAP\n", slice_type);
            return -1;
        }

        const int output_flag_present_flag = 0;
        if (output_flag_present_flag)
            skip_bits(gb, 1);   /* pic_output_flag */

        // [...] not separate colour plane

        if (nut != IDR_W_RADL && nut != IDR_N_LP) {
            if (!(inter_tools & INTER_TMVP)) {
                fprintf(stderr, "tmvp must be set unless all frames are IDR\n");
                return -1;
            }
            const uint32_t short_term_ref_pic_set_sps_flag = 0;
            const uint8_t prev_poc = *last_poc;

            /* log2_max_poc_lsb is defined as 8 in BPG */
            slice_pic_order_cnt_lsb = (uint8_t) get_bits(gb, 8);
            if ((slice_pic_order_cnt_lsb < prev_poc)
                && (256 + slice_pic_order_cnt_lsb < prev_poc)) {
                fprintf(stderr, "input is not monotonically ordered (%d < %d)\n",
                                 slice_pic_order_cnt_lsb, prev_poc);
                return -1;
            }

            if (get_bits(gb, 1) != short_term_ref_pic_set_sps_flag) {
                fprintf(stderr, "short_term_ref_pic_set_sps_flag should be %d\n",
                                 short_term_ref_pic_set_sps_flag);
                return -1;
            }

            if (!short_term_ref_pic_set_sps_flag) {
                uint32_t num_neg_pics, num_pos_pics;

                num_neg_pics = get_ue_golomb(gb);
                num_pos_pics = get_ue_golomb(gb);

                if (num_neg_pics > 1 || num_pos_pics > 0) {
                    fprintf(stderr, "too many neg (%d) or pos (%d) refs\n", num_neg_pics, num_pos_pics);
                    return -1;
                } else if (slice_type == 0) {
                    fprintf(stderr, "b-slice (2 MVs per block) in use\n");
                    return -1;
                }

                if (num_neg_pics) {
                    uint8_t delta_poc_s0_minus1, used_by_curr_pic_s0_flag;
                    int refed;

                    delta_poc_s0_minus1 = (uint8_t) get_ue_golomb(gb);
                    used_by_curr_pic_s0_flag = get_bits(gb, 1);

                    refed  = slice_pic_order_cnt_lsb - delta_poc_s0_minus1 - 1;
                    refed += 256 * (refed < 0);
                    if (used_by_curr_pic_s0_flag) {
                        if (inter_tools != INTER_ALL) {
                            if (!(inter_tools & INTER_AMP))
                                fprintf(stderr, "interpred used but AMP is not set\n");
                            if (!(inter_tools & INTER_TU_RQT))
                                fprintf(stderr, "interpred used but TU Inter depth doesn\'t equal Intra\n");
                            return -1;
                        }
                        if (delta_poc_s0_minus1 > 0 && refed != prev_poc)
                            fprintf(stderr, "warning: ref frame (%d) is not the immediate previous (%d)\n",
                                             refed, prev_poc);
                    }
                }
            }
        } else {
            /* decoder has been refreshed */
            slice_pic_order_cnt_lsb = 0;
        }
    }

    *last_poc = slice_pic_order_cnt_lsb;

    return 0;
}

/* suppress the VPS NAL and keep only the useful part of the SPS
   header. The decoder can rebuild a valid HEVC stream if needed. */
/* also read and store per-stream encode details */
static int prepare_headers(HEVCVideoConfig *plane,
                           const uint8_t *buf, const uint32_t buf_len)
{
    int idx, ret;
    uint32_t nal_len, msps_buf_len, out_buf_len, out_buf_len_max;
    uint8_t i, nut;
    uint8_t *nal_buf, *msps_buf, *out_buf, *p;
    GetBitState gb_s, *gb = &gb_s;
    PutBitState pb_s, *pb = &pb_s;

    /* these will be directly used to find video cfg settings */
    uint32_t width, height, bit_depth, fps_num, fps_den, min_cb_size;
    uint32_t video_full_range_flag, tiles, wpp, dependent_slices;
    HEVCInterToolsEnum inter_tools;
    BPGColorSpaceEnum color_space;
    BPGImageFormatEnum pixel_format;

    /* not all streams provide these, so create defaults */
    color_space = DEFAULT_COLORSPACE;
    inter_tools = INTER_ALL;
    video_full_range_flag = fps_num = fps_den = 0;

    out_buf = NULL;

    /* VPS NAL */
    idx = extract_nal(&nal_buf, &nal_len, buf, buf_len);
    free(nal_buf);
    nal_buf = NULL;
    if (idx < 0)
        goto bad_header_info;

    /* SPS NAL */
    ret = extract_nal(&nal_buf, &nal_len, buf + idx, buf_len);
    if (ret < 0)
        goto bad_header_info;
    else
        idx += ret;
    nut = (nal_buf[0] >> 1) & 0x3f;
    if (nut != 33) {
        fprintf(stderr, "expecting SPS nal, got (%d)\n", nut);
        goto bad_header_info;
    }

    /* skip the initial part of the SPS up to and including
       log2_min_cb_size */
    {
        uint32_t vps_id, max_sub_layers, sps_id;
        uint32_t chroma_format_idc, bit_depth_luma, bit_depth_chroma;
        uint32_t log2_max_poc_lsb, log2_min_cb_size;
        uint32_t log2_diff_max_min_coding_block_size, log2_min_tb_size;
        uint32_t log2_diff_max_min_transform_block_size;
        uint32_t max_transform_hierarchy_depth_inter;
        uint32_t max_transform_hierarchy_depth_intra;
        uint32_t scaling_list_enable_flag, amp_enabled_flag, sao_enabled;
        uint32_t pcm_enabled_flag, nb_st_rps;
        uint32_t long_term_ref_pics_present_flag, sps_strong_intra_smoothing_enable_flag, vui_present;
        uint32_t sps_temporal_mvp_enabled_flag;
        uint32_t pcm_sample_bit_depth_luma_minus1;
        uint32_t pcm_sample_bit_depth_chroma_minus1;
        uint32_t log2_min_pcm_luma_coding_block_size_minus3;
        uint32_t log2_diff_max_min_pcm_luma_coding_block_size;
        uint32_t pcm_loop_filter_disabled_flag;
        uint32_t sps_extension_flag, sps_range_extension_flag, sps_extension_7bits;
        uint32_t sps_range_extension_flags;

        init_get_bits(gb, nal_buf, nal_len);
        skip_bits(gb, 16); /* nal header */
        vps_id = get_bits(gb, 4);
        if (vps_id != 0) {
            fprintf(stderr, "VPS id 0 expected\n");
            goto bad_header_info;
        }
        max_sub_layers = get_bits(gb, 3);
        if (max_sub_layers != 0) {
            fprintf(stderr, "max_sub_layers == 0 expected\n");
            goto bad_header_info;
        }
        skip_bits(gb, 1); /* temporal_id_nesting_flag */
        /* profile tier level */
        skip_bits(gb, 2); /* profile_space */
        skip_bits(gb, 1); /* tier_flag */
        skip_bits(gb, 5); /* profile_idc */
        for(i = 0; i < 32; i++) {
            skip_bits(gb, 1); /* profile_compatibility_flag */
        }
        skip_bits(gb, 1); /* progressive_source_flag */
        skip_bits(gb, 1); /* interlaced_source_flag */
        skip_bits(gb, 1); /* non_packed_constraint_flag */
        skip_bits(gb, 1); /* frame_only_constraint_flag */
        skip_bits(gb, 44); /*  XXX_reserved_zero_44 */
        skip_bits(gb, 8); /* level_idc */

        sps_id = get_ue_golomb(gb);
        if (sps_id != 0) {
            fprintf(stderr, "SPS id 0 expected (%d)\n", sps_id);
            goto bad_header_info;
        }
        chroma_format_idc = get_ue_golomb(gb);
        pixel_format = chroma_format_idc;
        if (chroma_format_idc == 0)
            pixel_format = BPG_FORMAT_GRAY;
        else if (chroma_format_idc == 1)
            pixel_format = BPG_FORMAT_420_VIDEO;
        else if (chroma_format_idc == 2)
            pixel_format = BPG_FORMAT_422_VIDEO;
        else if (chroma_format_idc == 3) {
            pixel_format = BPG_FORMAT_444;
            /* separate_colour_plane_flag */
            if (get_bits(gb, 1) != 0) {
                fprintf(stderr, "separate_colour_plane_flag should be 0\n");
                goto bad_header_info;
            }
        }

        width = get_ue_golomb(gb);
        height = get_ue_golomb(gb);
        if ((width % 8) + (width % 8) != 0) {
            fprintf(stderr, "conformance dims (%d x %d) expected to be multiple of at least 8\n",
                             width, height);
            goto bad_header_info;
        }

        /* pic conformance_flag */
        if (get_bits(gb, 1)) {
            uint8_t h_scale, v_scale;
            /* pads of subsampled dimensions were right shifted. undo. */
            h_scale = (chroma_format_idc  % 3) ? 2 : 1;
            v_scale = (chroma_format_idc == 1) ? 2 : 1;

            /* undo picture padding */
            width -= get_ue_golomb(gb) * h_scale; /* left_offset */
            width -= get_ue_golomb(gb) * h_scale; /* right_offset */
            height -= get_ue_golomb(gb) * v_scale; /* top_offset */
            height -= get_ue_golomb(gb) * v_scale; /* bottom_offset */
        }
        if (width < 8 || height < 8) {
            fprintf(stderr, "unsupported width (%d) or height (%d)\n", width, height);
            goto bad_header_info;
        }

        bit_depth_luma = bit_depth = get_ue_golomb(gb) + 8;
        bit_depth_chroma = get_ue_golomb(gb) + 8;
        if (bit_depth_luma != bit_depth_chroma) {
            fprintf(stderr, "luma & chroma bit-depth must be equal\n");
            goto bad_header_info;
        }
        if (bit_depth_luma > BIT_DEPTH_MAX) {
            fprintf(stderr, "unsupported bit-depth\n");
            goto bad_header_info;
        }

        log2_max_poc_lsb = get_ue_golomb(gb) + 4;
        if (log2_max_poc_lsb != 8) {
            fprintf(stderr, "log2_max_poc_lsb must be 8 (%d)\n", log2_max_poc_lsb);
            goto bad_header_info;
        }
        skip_bits(gb, 1);   /* sps_sub_layer_ordering_info_present_flag */

        {
            uint32_t max_dec_pic_buffer, num_reorder_pics, max_latency_incr;

            max_dec_pic_buffer = get_ue_golomb(gb); /* max_dec_pic_buffering */
            num_reorder_pics = get_ue_golomb(gb);   /* num_reorder_pics */
            max_latency_incr = get_ue_golomb(gb);   /* max_latency_increase, 0 is infinite */
            if (max_dec_pic_buffer > 2 || num_reorder_pics > 1 || max_latency_incr > 1) {
                /* should be 1|0|1 instead of 2|1|1 but x265 does differently */
                fprintf(stderr, "warning: not marked as zero-delay (2 1 1 at most, got %d %d %d instead)\n",
                                 max_dec_pic_buffer, num_reorder_pics, max_latency_incr);
            }
        }

        log2_min_cb_size = get_ue_golomb(gb) + 3;
        min_cb_size = 1 << log2_min_cb_size;
        log2_diff_max_min_coding_block_size = get_ue_golomb(gb);
        log2_min_tb_size = get_ue_golomb(gb) + 2;
        log2_diff_max_min_transform_block_size = get_ue_golomb(gb);

        max_transform_hierarchy_depth_inter = get_ue_golomb(gb);
        max_transform_hierarchy_depth_intra = get_ue_golomb(gb);

        scaling_list_enable_flag = get_bits(gb, 1);
        if (scaling_list_enable_flag != 0) {
            fprintf(stderr, "scaling_list_enable_flag must be 0\n");
            goto bad_header_info;
        }
        amp_enabled_flag = get_bits(gb, 1);
        sao_enabled = get_bits(gb, 1);
        pcm_enabled_flag = get_bits(gb, 1);
        if (pcm_enabled_flag) {
            pcm_sample_bit_depth_luma_minus1 = get_bits(gb, 4);
            pcm_sample_bit_depth_chroma_minus1 = get_bits(gb, 4);
            log2_min_pcm_luma_coding_block_size_minus3 = get_ue_golomb(gb);
            log2_diff_max_min_pcm_luma_coding_block_size = get_ue_golomb(gb);
            pcm_loop_filter_disabled_flag = get_bits(gb, 1);
        }

        nb_st_rps = get_ue_golomb(gb);
        if (nb_st_rps != 0) {
            fprintf(stderr, "nb_st_rps must be 0 (%d)\n", nb_st_rps);
            goto bad_header_info;
        }
        long_term_ref_pics_present_flag = get_bits(gb, 1);
        if (long_term_ref_pics_present_flag) {
            fprintf(stderr, "nlong_term_ref_pics_present_flag must be 0 (%d)\n", nb_st_rps);
            goto bad_header_info;
        }
        sps_temporal_mvp_enabled_flag = get_bits(gb, 1);

        inter_tools =   INTER_TU_RQT * (max_transform_hierarchy_depth_inter == max_transform_hierarchy_depth_intra)
                      | INTER_TMVP * sps_temporal_mvp_enabled_flag
                      | INTER_AMP * amp_enabled_flag;
        if (inter_tools != INTER_ALL) {
            fprintf(stderr, "warning: invalid motion sps flags, so interprediction will not be allowed\n");
        }

        sps_strong_intra_smoothing_enable_flag = get_bits(gb, 1);
        vui_present = get_bits(gb, 1);
        if (vui_present) {
            uint32_t sar_present, sar_idx, overscan_info_present_flag;
            uint32_t video_signal_type_present_flag, chroma_loc_info_present_flag;
            uint32_t default_display_window_flag, vui_timing_info_present_flag;
            uint32_t vui_poc_proportional_to_timing_flag;
            uint32_t vui_hrd_parameters_present_flag, bitstream_restriction_flag;
            uint32_t colour_description_present_flag;

            sar_present = get_bits(gb, 1);
            if (sar_present) {
                sar_idx = get_bits(gb, 8);
                if (sar_idx == 255) {
                    skip_bits(gb, 16); /* sar_num */
                    skip_bits(gb, 16); /* sar_den */
                }
            }

            overscan_info_present_flag = get_bits(gb, 1);
            if (overscan_info_present_flag) {
                skip_bits(gb, 1); /* overscan_appropriate_flag */
            }

            video_signal_type_present_flag = get_bits(gb, 1);
            if (video_signal_type_present_flag) {
                if (get_bits(gb, 3) != 5 /* video_format */)
                    fprintf(stderr, "warning: video_format flag ignored\n");
                video_full_range_flag = get_bits(gb, 1);

                colour_description_present_flag = get_bits(gb, 1);
                if (colour_description_present_flag) {
                    uint32_t primaries, transfer, matrix;

                    primaries = get_bits(gb, 8);
                    transfer = get_bits(gb, 8); /* transfer_characteristics */
                    matrix = get_bits(gb, 8);   /* matrix_coeffs */

                    if (primaries == 1 || primaries == 2) {
                        color_space = BPG_CS_YCbCr_BT709;
                    } else if (primaries >= 5 && primaries <= 7) {
                        color_space = BPG_CS_YCbCr;
                    } else if (primaries == 9) {
                        color_space = BPG_CS_YCbCr_BT2020 + (matrix == 10);
                    } else {
                        fprintf(stderr, "warning: unsupported color space (%d)\n", primaries);
                        primaries = 2; // unspecified
                    }

                    if (transfer != 2 && transfer != 13)
                        fprintf(stderr, "warning: sRGB gamma will likely be assumed\n");

                    if (primaries == 2) {
                        if (matrix == 0) {
                            color_space = BPG_CS_RGB; // probably
                        } else if (matrix == 1 || matrix == 2) {
                            color_space = BPG_CS_YCbCr_BT709; // probably
                        } else if (matrix == 4 || matrix == 5 || matrix == 6) {
                            color_space = BPG_CS_YCbCr; // probably
                        } else if (matrix == 8) {
                            color_space = BPG_CS_YCgCo; // YCgCo
                        } else if (matrix == 9 || matrix == 10) {
                            color_space = matrix - 5; // YCbCr BT2020 NCL and CL
                        } else  {
                            fprintf(stderr, "unsupported color matrix coefficients (%d)\n", matrix);
                            goto bad_header_info;
                        }
                    }
                }
            }
            chroma_loc_info_present_flag = get_bits(gb, 1);
            if (chroma_loc_info_present_flag) {
                uint32_t loc_top, loc_bot;

                loc_top = get_ue_golomb(gb);
                loc_bot = get_ue_golomb(gb);
                if (chroma_format_idc == 1 && (loc_top == loc_bot)
                                           && (loc_top == 1)) {
                    pixel_format = BPG_FORMAT_420;
                }
            }
            skip_bits(gb, 1); /* neutra_chroma_indication_flag */
            skip_bits(gb, 1); /* field_seq_flag */
            skip_bits(gb, 1); /* frame_field_info_present_flag */
            default_display_window_flag = get_bits(gb, 1);
            if (default_display_window_flag) {
                fprintf(stderr, "default_display_window_flag must be 0\n");
                goto bad_header_info;
            }

            vui_timing_info_present_flag = get_bits(gb, 1);
            if (vui_timing_info_present_flag) {
                fps_den = get_bits_long(gb, 32); /* vui_num_units_in_tick */
                fps_num = get_bits_long(gb, 32); /* vui_time_scale */
                vui_poc_proportional_to_timing_flag = get_bits(gb, 1);
                if (vui_poc_proportional_to_timing_flag) {
                    get_ue_golomb(gb);
                }
                vui_hrd_parameters_present_flag = get_bits(gb, 1);
                if (vui_hrd_parameters_present_flag) {
                    fprintf(stderr, "vui_hrd_parameters_present_flag must be 0\n");
                    goto bad_header_info;
                }
            }
            bitstream_restriction_flag = get_bits(gb, 1);
            if (bitstream_restriction_flag) {
                skip_bits(gb, 1);
                skip_bits(gb, 1);
                skip_bits(gb, 1);
                get_ue_golomb(gb);
                get_ue_golomb(gb);
                get_ue_golomb(gb);
                get_ue_golomb(gb);
                get_ue_golomb(gb);
            }
        }
        sps_extension_flag = get_bits(gb, 1);
        sps_range_extension_flag = 0;
        sps_range_extension_flags = 0;
        if (sps_extension_flag) {
            sps_range_extension_flag = get_bits(gb, 1);
            sps_extension_7bits = get_bits(gb, 7);
            if (sps_extension_7bits != 0) {
                fprintf(stderr, "sps_extension_7bits must be 0\n");
                goto bad_header_info;
            }
            if (sps_range_extension_flag) {
                sps_range_extension_flags = get_bits(gb, 9);
                if (sps_range_extension_flags & ((1 << (8 - 3)) |
                                                 (1 << (8 - 4)) |
                                                 (1 << (8 - 6)) |
                                                 (1 << (8 - 8)))) {
                    fprintf(stderr, "unsupported range extensions (0x%x)\n",
                            sps_range_extension_flags);
                    goto bad_header_info;
                }
            }
        }

        /* build the modified SPS */
        msps_buf = malloc(nal_len + 32);
        memset(msps_buf, 0, nal_len + 16);

        init_put_bits(pb, msps_buf);
        put_ue_golomb(pb, log2_min_cb_size - 3);
        put_ue_golomb(pb, log2_diff_max_min_coding_block_size);
        put_ue_golomb(pb, log2_min_tb_size - 2);
        put_ue_golomb(pb, log2_diff_max_min_transform_block_size);
        put_ue_golomb(pb, max_transform_hierarchy_depth_intra);
        put_bits(pb, 1, sao_enabled);
        put_bits(pb, 1, pcm_enabled_flag);
        if (pcm_enabled_flag) {
            put_bits(pb, 4, pcm_sample_bit_depth_luma_minus1);
            put_bits(pb, 4, pcm_sample_bit_depth_chroma_minus1);
            put_ue_golomb(pb, log2_min_pcm_luma_coding_block_size_minus3);
            put_ue_golomb(pb, log2_diff_max_min_pcm_luma_coding_block_size);
            put_bits(pb, 1, pcm_loop_filter_disabled_flag);
        }
        put_bits(pb, 1, sps_strong_intra_smoothing_enable_flag);
        put_bits(pb, 1, sps_extension_flag);
        if (sps_extension_flag) {
            put_bits(pb, 1, sps_range_extension_flag);
            put_bits(pb, 7, 0);
            if (sps_range_extension_flag) {
                put_bits(pb, 9, sps_range_extension_flags);
            }
        }
        msps_buf_len = (pb->idx + 7) >> 3;

        out_buf_len_max = 5 + msps_buf_len;
        out_buf = malloc(out_buf_len_max);

        p = out_buf;
        put_ue(&p, msps_buf_len); /* header length */

        memcpy(p, msps_buf, msps_buf_len);
        p += msps_buf_len;

        out_buf_len = p - out_buf;
        free(msps_buf);
        free(nal_buf);
    }

    /* PPS NAL */
    ret = extract_nal(&nal_buf, &nal_len, buf + idx, buf_len);
    if (ret < 0)
        goto bad_header_info;
    nut = (nal_buf[0] >> 1) & 0x3f;
    if (nut != 34) {
        fprintf(stderr, "expecting PPS nal, got (%d)\n", nut);
        goto bad_header_info;
    }
    {
        init_get_bits(gb, nal_buf, nal_len);
        skip_bits(gb, 16);  // nal header

        get_ue_golomb(gb);  /* pps_pic_parameter_set_id */
        get_ue_golomb(gb);  /* pps_seq_parameter_set_id */

        dependent_slices = get_bits(gb, 1);

        if (get_bits(gb, 1) != 0) {
            fprintf(stderr, "output_flag_present_flag currently expected to equal 0\n");
            goto bad_header_info;
        }
        if (get_bits(gb, 3) != 0) {
            fprintf(stderr, "num_extra_slice_header_bits currently expected to equal 0\n");
            goto bad_header_info;
        }
        skip_bits(gb, 1);   /* sign_data_hiding_enabled_flag */
        skip_bits(gb, 1);   /* cabac_init_present_flag */
        get_ue_golomb(gb);  /* num_ref_idx_l0_default_active_minus1 */
        get_ue_golomb(gb);  /* num_ref_idx_l1_default_active_minus1 */
        get_ue_golomb(gb);  /* init_qp_minus26 */
        skip_bits(gb, 1);   /* constrained_intra_pred_flag */
        skip_bits(gb, 1);   /* transform_skip_enabled_flag */

        if (get_bits(gb, 1))    /* cu_qp_delta_enabled_flag */
            get_ue_golomb(gb);  /* diff_cu_qp_delta_depth */

        get_ue_golomb(gb);  /* pps_cb_qp_offset */
        get_ue_golomb(gb);  /* pps_cr_qp_offset */
        skip_bits(gb, 1);   /* pps_slice_chroma_qp_offsets_present_flag */
        skip_bits(gb, 1);   /* weighted_pred_flag */
        skip_bits(gb, 1);   /* weighted_bipred_flag */
        skip_bits(gb, 1);   /* transquant_bypass_enable_flag */

        tiles = get_bits(gb, 1);
        wpp = get_bits(gb, 1);

        if (tiles + wpp > 1) {
            fprintf(stderr, "simultaneous Tiles and WPP not supported\n");
            goto bad_header_info;
        }
    }
    free(nal_buf);
    nal_buf = NULL;

    /* save plane's configuration and header information */
    {
        HEVCVideoConfig *v;

        if (out_buf_len < 1 || idx < 1) {
            fprintf(stderr, "unexpected header size (%d) or location (%d) \n",
                             out_buf_len, idx);
            goto bad_header_info;
        }

        v = mallocz(sizeof(HEVCVideoConfig));
        if (!v)
            goto bad_header_info;

        /* store stream specific config */
        v->width = width;
        v->height = height;
        v->min_cb_size = min_cb_size;
        v->fps_num = fps_num;
        v->fps_den = fps_den;

        v->bit_depth = (uint8_t) bit_depth;
        v->limited_range = (uint8_t) !video_full_range_flag;
        v->pixel_format = pixel_format;
        v->color_space = color_space;

        v->inter_tools = inter_tools;
        v->parallel = wpp * PARALLEL_WPP | tiles * PARALLEL_TILES;
        v->dependent_slices = (uint8_t) dependent_slices;

        /* store actual modified sps */
        v->msps = out_buf;
        v->msps_len = out_buf_len;
        v->pps_idx = idx;

        *plane = *v;
    }

    return idx;
 bad_header_info:
    if (nal_buf)
        free(nal_buf);
    nal_buf = NULL;
    if (out_buf)
        free(out_buf);
    out_buf = NULL;
    return -1;

}

static uint32_t bpg_muxer_build_hevc(uint8_t **pout_buf, BPGMuxerContext *s,
                               const uint32_t cbuf_len,
                               const uint32_t abuf_len,
                               FILE *fdelay)
{
    DynBuf out_buf_s, *out_buf = &out_buf_s;
    const uint8_t *cbuf = s->color_buf;
    const uint8_t *abuf = s->alpha_buf;
    const uint8_t *nal_buf;

    int nal_len;
    uint32_t msps_len, cidx, aidx;
    uint8_t nut, last_poc, is_alpha, first_nal, start, l;
    HEVCInterToolsEnum inter_tools;

    cidx = aidx = last_poc = 0;
    inter_tools = s->color.inter_tools;

    dyn_buf_init(out_buf);

    /* add alpha MSPS */
    if (abuf) {
        inter_tools &= s->alpha.inter_tools;

        aidx = s->alpha.pps_idx;
        msps_len = s->alpha.msps_len;
        if (cidx < 16 || msps_len < 2) {
            fprintf(stderr, "alpha sps size unexpectedly small %d %d\n", aidx, msps_len);
            goto fail;
        }

        memcpy(out_buf->buf + out_buf->len, s->alpha.msps, msps_len);
        out_buf->len += msps_len;
        free(s->alpha.msps);
        s->alpha.msps = NULL;
    }

    /* add color MSPS */
    cidx = s->color.pps_idx;
    msps_len = s->color.msps_len;
    if (cidx < 16 || msps_len < 2) {
        fprintf(stderr, "color sps size unexpectedly small %d %d\n", cidx, msps_len);
        goto fail;
    }
    if (dyn_buf_resize(out_buf, out_buf->len + msps_len) < 0)
        goto fail;

    memcpy(out_buf->buf + out_buf->len, s->color.msps, msps_len);
    out_buf->len += msps_len;
    free(s->color.msps);
    s->color.msps = NULL;

    /* add the remaining NALs, alternating between alpha (if present)
       and color. */
    is_alpha = (abuf != NULL);
    first_nal = 1;
    for(;;) {
        if (s->frame_count >= s->frame_limit) {
            break;
        }
        if (!is_alpha) {
            if (cidx >= cbuf_len) {
                if (abuf) {
                    fprintf(stderr, "Incorrect number of alpha NALs\n");
                    goto fail;
                }
                break;
            }
            nal_buf = cbuf + cidx;
            nal_len = find_nal_end(nal_buf, cbuf_len - cidx);
            if (nal_len < 0)
                goto fail;
            cidx += nal_len;
        } else {
            if (aidx >= abuf_len)
                break;
            nal_buf = abuf + aidx;
            nal_len = find_nal_end(nal_buf, abuf_len - aidx);
            if (nal_len < 0)
                goto fail;
            aidx += nal_len;
        }

        /* check that input NAL types are expected and valid */
        /* the parser will double check that slice type and ref pix are within spec */
        start = 3 + (nal_buf[2] == 0);
        nut = (nal_buf[start] >> 1) & 0x3f;
        if (nut >= AUD)
            continue;
        if ( !((start + 2 < nal_len) &&
               (parse_slice(nal_buf + start, nal_len, nut, &s->color, inter_tools, &last_poc) >= 0)) ) {
            fprintf(stderr, "error while parsing (alpha? %d) NALs\n", is_alpha);
            goto fail;
        }

        /* store animation data for displayed frames */
        if (!is_alpha && nut <= RSV_IRAP) {
            /* if delay file exists, use delay timings from there for each new frame */
            if (fdelay) {
                double fdelay_val, ticks;
                if (fscanf(fdelay, "%lf", &fdelay_val) == 1) {
                    ticks = lrint(fdelay_val * s->frame_delay_den / (s->frame_delay_num * 100));
                    if (ticks >= 1 && ticks <= USHRT_MAX)
                        s->frame_ticks = (uint16_t)ticks;
                    else if (ticks > USHRT_MAX)
                        s->frame_ticks = USHRT_MAX;
                }
            }

            uint16_t frame_ticks = s->frame_ticks;
            /* store the frame duration */
            if ((s->frame_count + 1) > s->frame_duration_tab_size) {
                uint16_t tab_sz = s->frame_duration_tab_size;

                if (tab_sz > SHRT_MAX)
                    tab_sz = USHRT_MAX;
                else
                    tab_sz = (tab_sz * 3) / 2;

                if (tab_sz < (s->frame_count + 1))
                    tab_sz = (s->frame_count + 1);

                s->frame_duration_tab = realloc(s->frame_duration_tab,
                                                sizeof(s->frame_duration_tab) * tab_sz);
                s->frame_duration_tab_size = tab_sz;
            }
            s->frame_duration_tab[s->frame_count] = frame_ticks;

            /* add SEI NAL for the frame duration (animation case) */
            if (frame_ticks > 1) {
                add_frame_duration_sei(out_buf, frame_ticks);
            }
            s->frame_count++;
        }

        if (first_nal) {
            /* skip first start code */
            l = start;
        } else {
            l = 0;
        }
        if (dyn_buf_resize(out_buf, out_buf->len + nal_len - l) < 0)
            goto fail;
        memcpy(out_buf->buf + out_buf->len, nal_buf + l, nal_len - l);
        if (is_alpha) {
            /* set nul_layer_id of alpha to '1' */
            out_buf->buf[out_buf->len + (start - l) + 1] |= 1 << 3;
        }
        out_buf->len += nal_len - l;

        if (abuf) {
            is_alpha ^= 1;
        }
        first_nal = 0;
    }
    *pout_buf = out_buf->buf;
    return out_buf->len;
 fail:
    free(out_buf->buf);
    out_buf->buf = NULL;
    if (s->frame_duration_tab) {
        free(s->frame_duration_tab);
        s->frame_duration_tab = NULL;
    }
    return 0;
}

static BPGMuxerContext *bpg_muxer_open()
{
    BPGMuxerContext *s;

    s = mallocz(sizeof(BPGMuxerContext));
    if (!s)
        return NULL;
    s->color_buf = NULL;
    s->alpha_buf = NULL;

    s->loop_count = 0;
    s->frame_delay_num = 0;
    s->frame_delay_den = 0;

    s->frame_count = 0;
    s->frame_ticks = 1;
    s->frame_duration_tab = NULL;
    s->frame_duration_tab_size = 0;

    s->frame_limit = USHRT_MAX;

    s->color_space = BPG_CS_COUNT;
    s->limited_range = 255;
    s->premultiplied_alpha = 0;
    s->cmyk = 0;
    return s;
}

static int bpg_muxer_finish_ext(BPGMuxerContext *s, void *opaque) {
    uint8_t *extension_buf;
    uint32_t extension_buf_len;

    if (s->frame_count > 1) {
        BPGMetaData *md;
        uint8_t buf[15], *q;

        md = bpg_md_alloc(BPG_EXTENSION_TAG_ANIM_CONTROL);
        q = buf;
        put_ue(&q, s->loop_count);
        put_ue(&q, s->frame_delay_num);
        put_ue(&q, s->frame_delay_den);
        md->buf_len = q - buf;
        md->buf = malloc(md->buf_len);
        memcpy(md->buf, buf, md->buf_len);
        md->next = s->first_md;
        s->first_md = md;
    }

    extension_buf = NULL;
    extension_buf_len = 0;
    if (s->first_md) {
        BPGMetaData *md1;
        uint32_t max_len;
        uint8_t *q;

        max_len = 0;
        for(md1 = s->first_md; md1 != NULL; md1 = md1->next) {
            max_len += md1->buf_len + 5 * 2;
        }
        extension_buf = malloc(max_len);
        q = extension_buf;
        for(md1 = s->first_md; md1 != NULL; md1 = md1->next) {
            put_ue(&q, md1->tag);
            put_ue(&q, md1->buf_len);
            memcpy(q, md1->buf, md1->buf_len);
            q += md1->buf_len;
        }
        extension_buf_len = q - extension_buf;

        bpg_md_free(s->first_md);
        s->first_md = NULL;
    }

    {
        uint8_t img_header[128], *q;
        uint8_t v, has_alpha, has_extension, alpha2_flag, alpha1_flag, format;

        has_alpha = (s->alpha_buf != NULL && s->alpha.msps_len > 0);
        has_extension = (extension_buf_len > 0);

        if (has_alpha) {
            if (s->cmyk) {
                alpha1_flag = 0;
                alpha2_flag = 1;
            } else {
                alpha1_flag = 1;
                alpha2_flag = s->premultiplied_alpha;
            }
        } else {
            alpha1_flag = 0;
            alpha2_flag = 0;
        }

        q = img_header;
        *q++ = (IMAGE_HEADER_MAGIC >> 24) & 0xff;
        *q++ = (IMAGE_HEADER_MAGIC >> 16) & 0xff;
        *q++ = (IMAGE_HEADER_MAGIC >> 8) & 0xff;
        *q++ = (IMAGE_HEADER_MAGIC >> 0) & 0xff;


        format = s->color.pixel_format;
        v = (format << 5) | (alpha1_flag << 4) | (s->color.bit_depth - 8);
        *q++ = v;
        v = (s->color_space << 4) | (has_extension << 3) |
            (alpha2_flag << 2) | (s->limited_range << 1) |
            (s->frame_count > 1);//
        *q++ = v;
        put_ue(&q, s->color.width);
        put_ue(&q, s->color.height);

        put_ue(&q, 0); /* zero length means up to the end of the file */
        if (has_extension) {
            put_ue(&q, extension_buf_len); /* extension data length */
        }

        fwrite(img_header, 1, q - img_header, opaque);

        if (has_extension) {
            if (fwrite(extension_buf, 1, extension_buf_len, opaque) != extension_buf_len) {
                fprintf(stderr, "Error while writing extension data\n");
                free(extension_buf);
                return -1;
            }
            free(extension_buf);
        }
    }

    return 0;
}

static uint32_t bpg_muxer_load_hevc(uint8_t **pbuf, HEVCVideoConfig *cfg, const char *hevc_name)
{
    FILE *f;
    uint8_t *out_buf;
    uint32_t out_buf_len;

    out_buf = NULL;

    f = fopen(hevc_name, "rb");
    if (!f) {
        fprintf(stderr, "Issue opening %s\n", hevc_name);
        goto hevc_load_error;
    }
    /* very casual check to make sure input is raw bistream */
    {
        uint8_t check[5];
        if (fread(check, 1, 5, f) != 5
            ||  0 != (check[0] + check[1] + check[2])
            ||  1 != check[3]
            || 64 != check[4]) {
            fprintf(stderr, "Input is not raw HEVC bitstream\n");
            goto hevc_load_error;
        }
    }

    fseek(f, 0, SEEK_END);
    out_buf_len = ftell(f);
    if (out_buf_len > 1<<29) {
    	fprintf(stderr, "input size larger than reasonable (512 MB)\n");
    	fprintf(stderr, "reading in chunks might be supported in some indefinite future\n");
    	goto hevc_load_error;
    }

    fseek(f, 0, SEEK_SET);
    out_buf = (uint8_t *)malloc(out_buf_len);
    if (fread(out_buf, 1, out_buf_len, f) != out_buf_len) {
        fprintf(stderr, "Error while reading input HEVC\n");
        goto hevc_load_error;
    }

    if (prepare_headers(cfg, out_buf, out_buf_len) < 0) {
        fprintf(stderr, "Error preparing headers for %s\n", hevc_name);
        goto hevc_load_error;
    }

    fclose(f);
    *pbuf = out_buf;

    return out_buf_len;
  hevc_load_error:
    if (f)
        fclose(f);
    if (out_buf) {
        free(out_buf);
        out_buf = NULL;
    }
    return 0;
}

/* transform fps double to valid 16bit pair of delay numbers */
static void fps_dbl_to_delay(double fnum, uint16_t *delay_num, uint16_t *delay_den) {
    uint16_t fden = 1;

    for (;;) {
        if (fnum > SHRT_MAX || fden > SHRT_MAX)
            break;
        if (fabs(round(fnum) - fnum) < 0.0001)
            break;
        fnum *= 2;
        fden *= 2;
    }

    *delay_num = fden;
    *delay_den = (uint16_t) lrint(fnum);
}

static int check_hevc_cfg(BPGMuxerContext *s)
{
    const HEVCVideoConfig *color, *alpha;
    color = &s->color;
    alpha = &s->alpha;

    if (s->limited_range != 0 && s->limited_range != 1) {
        s->limited_range = color->limited_range;
    }

    if (s->frame_delay_num < 1 || s->frame_delay_den < 1) {
        if (color->fps_num && color->fps_den) {
            if (color->fps_num <= USHRT_MAX && color->fps_den <= USHRT_MAX) {
                /* delay is 1/fps */
                s->frame_delay_num = (uint16_t) color->fps_den;
                s->frame_delay_den = (uint16_t) color->fps_num;
            } else {
                fps_dbl_to_delay((color->fps_num / (double) color->fps_den),
                                 &s->frame_delay_num, &s->frame_delay_den);
            }
        } else {
            s->frame_delay_num = 1;
            s->frame_delay_den = 25;
        }
    }

    if (s->alpha_buf) {
        if (   alpha->width     != color->width
            || alpha->height    != color->height
            || alpha->bit_depth != color->bit_depth
            || alpha->parallel  != color->parallel)
        {
            fprintf(stderr, "color and alpha planes do not have similar characteristics\n");
            return -1;
        }
        if (alpha->limited_range) {
            fprintf(stderr, "warning: alpha plane should be encoded with full range\n");
        }
        if (alpha->pixel_format != BPG_FORMAT_GRAY) {
            fprintf(stderr, "alpha plane must be monochrome\n");
            return -1;
        }
    } else {
        if (s->premultiplied_alpha || s->cmyk) {
            fprintf(stderr, "premul or cmyk format indicated, but there's no alpha plane\n");
            return -1;
        }
    }

    if (s->color_space == BPG_CS_COUNT) {
        s->color_space = color->color_space;
    }
    return 0;
}

static void bpg_muxer_close(BPGMuxerContext *s)
{
    bpg_md_free(s->first_md);

    if (s->color.msps)
        free(s->color.msps);

    if (s->alpha.msps)
        free(s->alpha.msps);

    free(s->color_buf);
    free(s->alpha_buf);

    free(s->frame_duration_tab);
    free(s);
}

static void mux_help() __attribute__ ((noreturn));
static void mux_help(int is_full)
{
    printf("BPG Muxer version " CONFIG_BPG_VERSION "\n"
           "usage: bpgmux [options] infile.[h265|hevc]\n"
           "\n"
           "Inputs raw bitstreams should be monotonically ordered with\n"
           "at most one reference frame\n"
           "\n"
           "Main options:\n"
           "-h                   show the full help (including the advanced options)\n"
           "-a alpha_fname       name of 4th plane raw bitstream for muxing\n"
           "-o muxed_fname       set output filename (default = %s)\n"
           "-c color_space       set the input color space (ycbcr, rgb, ycgco,\n"
           "                         ycbcr_bt709, ycbcr_bt2020, default=ycbcr_bt709)\n"
           "\n"
           "Animation options:\n"
           "-fps N               set the frame rate\n"
           "                         (default = 25 or obtained from bitstream flags if available)\n"
           "-frames N            mux up to this many frames (default and max = %d)\n"
           "-loop N              set the number of times the animation is played. 0 means\n"
           "                         infinite (default = 0)\n"
           "-delayfile file      text file containing one number per image giving the\n"
           "                         display delay per image in centiseconds.\n"
           , DEFAULT_OUTFILENAME, USHRT_MAX);

    if (is_full) {
        printf("\nAdvanced options:\n"
           "-premul              set if input is already premultiplied with alpha (0 or 1, default 0)\n"
           "-limitedrange        indicate whether color input has full or limited pixel values\n"
           "				         (0 or 1, default 1, very likely 1)\n"
           "\n\n");
    }

    exit(1);
}

static struct option mux_adv_opts[] = {
    { "limitedrange", required_argument },
    { "premul", no_argument },
    { "loop", required_argument },
    { "fps", required_argument },
    { "frames", required_argument },
    { "delayfile", required_argument },
    { NULL },
};

int main(int argc, char **argv)
{
    const char *color_fname, *alpha_fname, *muxed_fname, *frame_delay_file;;
    FILE *f, *fdelay;
    BPGMetaData *md;
    uint8_t *muxed_buf;

    int c, option_index;
    uint32_t color_buf_len, alpha_buf_len;
    uint32_t muxed_buf_len;

    BPGMuxerContext *s;
    s = bpg_muxer_open();

    muxed_fname = DEFAULT_OUTFILENAME;
    frame_delay_file = NULL;
    alpha_fname = NULL;
    md = NULL;
    muxed_buf = NULL;

    color_buf_len = alpha_buf_len = 0;

    for(;;) {
        c = getopt_long_only(argc, argv, "a:o:c:h", mux_adv_opts, &option_index);
        if (c == -1)
            break;
        switch(c) {
        case 0:
            switch(option_index) {
            case 0:
                s->limited_range = !!strtol(optarg, NULL, 0);
                break;
            case 1:
                s->premultiplied_alpha = 1;
                break;
            case 2:
                sscanf(optarg, "%hu", &s->loop_count);
                break;
            case 3:
                if (2 != sscanf(optarg, "%hu/%hu", &s->frame_delay_den, &s->frame_delay_num)) {
                    double fps = atof(optarg);
                    if (fps > 0) {
                        fps_dbl_to_delay(fps, &s->frame_delay_num, &s->frame_delay_den);
                    }
                }
                if (s->frame_delay_den < 1 || s->frame_delay_num < 1) {
                    fprintf(stderr, "invalid frame rate\n");
                    exit(1);
                }
                break;
            case 4:
                if (sscanf(optarg, "%hu", &s->frame_limit) != 1) {
                    fprintf(stderr, "invalid frame limit\n");
                    exit(1);
                }
                break;
            case 5:
                frame_delay_file = optarg;
                break;
            default:
                goto show_help;
            }
            break;
        case 'a':
            alpha_fname = optarg;
            break;
        case 'o':
            muxed_fname = optarg;
            break;
        case 'c':
            if (!strcmp(optarg, "ycbcr")) {
                s->color_space = BPG_CS_YCbCr;
            } else if (!strcmp(optarg, "rgb")) {
                s->color_space = BPG_CS_RGB;
            } else if (!strcmp(optarg, "ycgco")) {
                s->color_space = BPG_CS_YCgCo;
            } else if (!strcmp(optarg, "ycbcr_bt709")) {
                s->color_space = BPG_CS_YCbCr_BT709;
            } else if (!strcmp(optarg, "ycbcr_bt2020")) {
                s->color_space = BPG_CS_YCbCr_BT2020;
            } else {
                fprintf(stderr, "Invalid color space format\n");
                exit(1);
            }
            break;
        case 'h':
          show_help:
            mux_help(1);
            break;
        default:
            exit(1);
        }
    }

    if (optind >= argc)
        mux_help(0);
    color_fname = argv[optind];

    f = fopen(muxed_fname, "wb");
    if (!f) {
        perror(muxed_fname);
        goto bpgmux_fail;
    }

    /* load input bitstreams into memory. also check per-bistream conformity */
    color_buf_len = bpg_muxer_load_hevc(&s->color_buf, &s->color, color_fname);
    if (color_buf_len < 1)
        goto bpgmux_fail;
    if (alpha_fname) {
        alpha_buf_len = bpg_muxer_load_hevc(&s->alpha_buf, &s->alpha, alpha_fname);
        if (alpha_buf_len < 1)
            goto bpgmux_fail;
    }
    /* make sure stored muxer settings and color and alpha agree */
    if (check_hevc_cfg(s) < 0)
        goto bpgmux_fail;

    if (frame_delay_file) {
        fdelay = fopen(frame_delay_file, "r");
        if (!fdelay) {
            fprintf(stderr, "Could not open '%s'\n", frame_delay_file);
            goto bpgmux_fail;
        }
    } else {
        fdelay = NULL;
    }

    s->first_md = md;

    /* start actual muxing process */
    muxed_buf_len = bpg_muxer_build_hevc(&muxed_buf, s, color_buf_len, alpha_buf_len, fdelay);
    if (muxed_buf_len < 1) {
        fprintf(stderr, "Error while building modified hevc buffer with %d frames processed\n",
        				 s->frame_count);
        goto bpgmux_fail;
    }

    /* start the writing process */
    if (bpg_muxer_finish_ext(s, f) < 0) {
        fprintf(stderr, "Error while writing extension data to disk\n");
        goto bpgmux_fail;
    }

    if (fwrite(muxed_buf, 1, muxed_buf_len, f) != muxed_buf_len) {
        fprintf(stderr, "Error while writing HEVC data to disk\n");
        goto bpgmux_fail;
    }

    fclose(f);

    bpg_muxer_close(s);
    exit(0);

  bpgmux_fail:
    fprintf(stderr, "Muxing has failed.\n");
    if (muxed_buf)
        free(muxed_buf);
    if (f)
        unlink(muxed_fname);
    bpg_muxer_close(s);
    exit(1);
}



