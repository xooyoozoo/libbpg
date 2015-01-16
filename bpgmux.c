#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <limits.h>
#include <getopt.h>
#include <math.h>

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

typedef struct BPGMetaData {
    uint32_t tag;
    uint8_t *buf;
    int buf_len;
    struct BPGMetaData *next;
} BPGMetaData;


typedef enum {
    HEVC_PARAL_NONE,
    HEVC_PARAL_TILES,
    HEVC_PARAL_WPP,

    HEVC_PARAL_COUNT,
} HEVCParallelismEnum;

typedef struct HEVCVideoConfig {
    int width;
    int height;
    int min_cb_size;
    int fps_num;
    int fps_den;

    int bit_depth;
    int limited_range;
    BPGImageFormatEnum pixel_format;
    BPGColorSpaceEnum color_space;

    HEVCParallelismEnum parallel;
    int dependent_slices;

    uint8_t *msps;
    int msps_len;
    int pps_idx;
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

    int frame_count;
    int frame_ticks;
    uint16_t *frame_duration_tab;
    int frame_duration_tab_size;

    BPGColorSpaceEnum color_space;
    int limited_range;
    int premultiplied_alpha;
    int cmyk;
} BPGMuxerContext;

BPGMetaData *bpg_md_alloc(uint32_t tag)
{
    BPGMetaData *md;
    md = malloc(sizeof(BPGMetaData));
    memset(md, 0, sizeof(*md));
    md->tag = tag;
    return md;
}

void bpg_md_free(BPGMetaData *md)
{
    BPGMetaData *md_next;

    while (md != NULL) {
        md_next = md->next;
        free(md->buf);
        free(md);
        md = md_next;
    }
}

void *mallocz(size_t size)
{
    void *ptr;
    ptr = malloc(size);
    if (!ptr)
        return NULL;
    memset(ptr, 0, size);
    return ptr;
}

/* return the position of the end of the NAL or -1 if error */
static int find_nal_end(const uint8_t *buf, int buf_len)
{
    int idx;

    idx = 0;
    if (buf_len >= 4 &&
        buf[0] == 0 && buf[1] == 0 && buf[2] == 0 && buf[3] == 1) {
        idx = 4;
    } else if (buf_len >= 3 &&
               buf[0] == 0 && buf[1] == 0 && buf[2] == 1) {
        idx = 3;
    } else {
        return -1;
    }
    /* NAL header */
    if (idx + 2 > buf_len)
        return -1;
    /* find the last byte */
    for(;;) {
        if (idx + 2 >= buf_len) {
            idx = buf_len;
            break;
        }
        if (buf[idx] == 0 && buf[idx + 1] == 0 && buf[idx + 2] == 1)
            break;
        if (idx + 3 < buf_len &&
            buf[idx] == 0 && buf[idx + 1] == 0 && buf[idx + 2] == 0 && buf[idx + 3] == 1)
            break;
        idx++;
    }
    return idx;
}

/* return the position of the end of the NAL or -1 if error */
static int extract_nal(uint8_t **pnal_buf, int *pnal_len,
                       const uint8_t *buf, int buf_len)
{
    int idx, start, end, len;
    uint8_t *nal_buf;
    int nal_len;

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

/* big endian variable length 7 bit encoding */
static void put_ue(uint8_t **pp, uint32_t v)
{
    uint8_t *p = *pp;
    int i, j;

    for(i = 1; i < 5; i++) {
        if (v < (1 << (7 * i)))
            break;
    }
    for(j = i - 1; j >= 1; j--)
        *p++ = ((v >> (7 * j)) & 0x7f) | 0x80;
    *p++ = v & 0x7f;
    *pp = p;
}

typedef struct {
    const uint8_t *buf;
    int idx;
    int buf_len;
} GetBitState;

static void init_get_bits(GetBitState *s, const uint8_t *buf, int buf_len)
{
    s->buf = buf;
    s->buf_len = buf_len;
    s->idx = 0;
}

static void skip_bits(GetBitState *s, int n)
{
    s->idx += n;
}

/* 1 <= n <= 25. return '0' bits if past the end of the buffer. */
static uint32_t get_bits(GetBitState *s, int n)
{
    const uint8_t *buf = s->buf;
    int p, i;
    uint32_t v;

    p = s->idx >> 3;
    if ((p + 3) < s->buf_len) {
        v = (buf[p] << 24) | (buf[p + 1] << 16) |
            (buf[p + 2] << 8) | buf[p + 3];
    } else {
        v = 0;
        for(i = 0; i < 3; i++) {
            if ((p + i) < s->buf_len)
                v |= buf[p + i] << (24 - i * 8);
        }
    }
    v = (v >> (32 - (s->idx & 7) - n)) & ((1 << n) - 1);
    s->idx += n;
    return v;
}

/* 1 <= n <= 32 */
static uint32_t get_bits_long(GetBitState *s, int n)
{
    uint32_t v;

    if (n <= 25) {
        v = get_bits(s, n);
    } else {
        n -= 16;
        v = get_bits(s, 16) << n;
        v |= get_bits(s, n);
    }
    return v;
}

/* at most 32 bits are supported */
static uint32_t get_ue_golomb(GetBitState *s)
{
    int i;
    i = 0;
    for(;;) {
        if (get_bits(s, 1))
            break;
        i++;
        if (i == 32)
            return 0xffffffff;
    }
    if (i == 0)
        return 0;
    else
        return ((1 << i) | get_bits_long(s, i)) - 1;
}

typedef struct {
    uint8_t *buf;
    int idx;
} PutBitState;

static void init_put_bits(PutBitState *s, uint8_t *buf)
{
    s->buf = buf;
    s->idx = 0;
}

static void put_bit(PutBitState *s, int bit)
{
    s->buf[s->idx >> 3] |= bit << (7 - (s->idx & 7));
    s->idx++;
}

static void put_bits(PutBitState *s, int n, uint32_t v)
{
    int i;

    for(i = 0; i < n; i++) {
        put_bit(s, (v >> (n - 1 - i)) & 1);
    }
}

static void put_ue_golomb(PutBitState *s, uint32_t v)
{
    uint32_t a;
    int n;

    v++;
    n = 0;
    a = v;
    while (a != 0) {
        a >>= 1;
        n++;
    }
    if (n > 1)
        put_bits(s, n - 1, 0);
    put_bits(s, n, v);
}

typedef struct {
    uint8_t *buf;
    int size;
    int len;
} DynBuf;

static void dyn_buf_init(DynBuf *s)
{
    s->buf = NULL;
    s->size = 0;
    s->len = 0;
}

static int dyn_buf_resize(DynBuf *s, int size)
{
    int new_size;
    uint8_t *new_buf;

    if (size <= s->size)
        return 0;
    new_size = (s->size * 3) / 2;
    if (new_size < size)
        new_size = size;
    new_buf = realloc(s->buf, new_size);
    if (!new_buf)
        return -1;
    s->buf = new_buf;
    s->size = new_size;
    return 0;
}

static int parse_slice(const uint8_t *buf, const int len, const int nut, const HEVCVideoConfig *cfg)
{
    int is_IRAP, first_slice_segment_in_pic;
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

    first_slice_segment_in_pic = get_bits(gb, 1); 
    if (is_IRAP) {
        skip_bits(gb, 1);
        /*if (get_bits(gb, 1) == 0) {
            fprintf(stderr, "no_output_of_prior_pics_flag should be 1 for IRAPs\n");
            return -1;
        }*/
    }
    if (get_ue_golomb(gb) != 0) {
        fprintf(stderr, "slice_pic_parameter_set_id should be 0\n");
        return -1;
    }

    if (!first_slice_segment_in_pic) {
        int slice_segment_addr_len;
        int w1, h1;

        if (cfg->dependent_slices) 
            if (get_bits(gb, 1))
                return 0;
        
        /* add back padding to get to (multiple of min_cb_size) */
        w1 = (cfg->width  + cfg->min_cb_size - 1) & ~(cfg->min_cb_size - 1);
        h1 = (cfg->height + cfg->min_cb_size - 1) & ~(cfg->min_cb_size - 1);
        slice_segment_addr_len = lrint(ceil(log(w1 * h1) / log(2)));

        get_bits(gb, slice_segment_addr_len); /* slice_segment_address */
    }

    if (!cfg->dependent_slices) {
        int i, slice_type;

        const int num_extra_slice_header_bits = 0;
        for (i = 0; i < num_extra_slice_header_bits; i++)
            skip_bits(gb, 1);   /* slice_reserved_flag[ i ] */

        slice_type = get_ue_golomb(gb);
        if (slice_type < 1 || slice_type > 2) {
            fprintf(stderr, "slice_type (%d) should be I or P\n", slice_type);
            return -1;
        }
        if (slice_type != 2 && is_IRAP) {
            fprintf(stderr, "slice_type (%d) expected to be IRAP\n", slice_type);
            return -1;
        }

        const int output_flag_present_flag = 0;
        if (output_flag_present_flag)
            skip_bits(gb, 1);   /* pic_output_flag */

        // [...] not separate colour plane
        
        if (nut != IDR_W_RADL && nut != IDR_N_LP) {
            /* log2_max_poc_lsb is defined as 8 in BPG */
            skip_bits(gb, 8);
            
            const int short_term_ref_pic_set_sps_flag = 0;
            if (get_bits(gb, 1) != short_term_ref_pic_set_sps_flag) {
                fprintf(stderr, "short_term_ref_pic_set_sps_flag should be 0\n");
                return -1;
            }

            if (!short_term_ref_pic_set_sps_flag) {
                int numNegPics, numPosPics;
                
                numNegPics = get_ue_golomb(gb);
                numPosPics = get_ue_golomb(gb);

                if (numNegPics > 1 || numPosPics > 0) {
                    fprintf(stderr, "too many neg (%d) or pos (%d) refs\n", numNegPics, numPosPics);
                    return -1;
                }

                if (numNegPics) {
                    /* delta_poc_s0_minus1 */
                    if (get_ue_golomb(gb) != 0) {
                        fprintf(stderr, "delta_poc_s0_minus1 should be 0\n");
                    }
                    skip_bits(gb, 1);   /* used_by_curr_pic_s0_flag */
                }
            }
        }
    }


    return 0;
}

/* suppress the VPS NAL and keep only the useful part of the SPS
   header. The decoder can rebuild a valid HEVC stream if needed. */
/* also read and store per-stream encode details */
static int prepare_headers(HEVCVideoConfig *plane,
                           const uint8_t *buf, const int buf_len)
{
    int nal_unit_type, nal_len, idx, i, ret, msps_buf_len;
    int out_buf_len, out_buf_len_max;
    uint8_t *nal_buf, *msps_buf, *out_buf;
    GetBitState gb_s, *gb = &gb_s;
    PutBitState pb_s, *pb = &pb_s;
    uint8_t *p;

    /* these will be directly used to find video cfg settings */
    int width, height, min_cb_size, bit_depth;
    int video_full_range_flag;
    int color_space, pixel_format;
    int fps_num, fps_den;
    int tiles, wpp, dependent_slices;

    /* not all streams provide these, so create defaults */
    color_space = DEFAULT_COLORSPACE;
    video_full_range_flag = 1;
    fps_num = fps_den = 0;

    out_buf = NULL;

    /* VPS NAL */
    idx = extract_nal(&nal_buf, &nal_len, buf, buf_len);
    free(nal_buf);
    if (idx < 0)
        goto bad_header_info;

    /* SPS NAL */
    idx += extract_nal(&nal_buf, &nal_len, buf + idx, buf_len);
    if (idx < 0)
        goto bad_header_info;
    nal_unit_type = (nal_buf[0] >> 1) & 0x3f;
    if (nal_unit_type != 33) {
        fprintf(stderr, "expecting SPS nal, got (%d)\n", nal_unit_type);
        goto bad_header_info;
    }

    /* skip the initial part of the SPS up to and including
       log2_min_cb_size */
    {
        int vps_id, max_sub_layers, profile_idc, sps_id;
        int chroma_format_idc, bit_depth_luma, bit_depth_chroma;
        int log2_max_poc_lsb, sublayer_ordering_info, log2_min_cb_size;
        int log2_diff_max_min_coding_block_size, log2_min_tb_size;
        int log2_diff_max_min_transform_block_size;
        int max_transform_hierarchy_depth_inter;
        int max_transform_hierarchy_depth_intra;
        int scaling_list_enable_flag, amp_enabled_flag, sao_enabled;
        int pcm_enabled_flag, nb_st_rps;
        int long_term_ref_pics_present_flag, sps_strong_intra_smoothing_enable_flag, vui_present;
        int sps_temporal_mvp_enabled_flag;
        int pcm_sample_bit_depth_luma_minus1;
        int pcm_sample_bit_depth_chroma_minus1;
        int log2_min_pcm_luma_coding_block_size_minus3;
        int log2_diff_max_min_pcm_luma_coding_block_size;
        int pcm_loop_filter_disabled_flag;
        int sps_extension_flag, sps_range_extension_flag, sps_extension_7bits;
        int sps_range_extension_flags;

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
        profile_idc = get_bits(gb, 5);
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
            if (get_bits(gb, 1) != 0)
                fprintf(stderr, "separate_colour_plane_flag should be 0\n");
        }

        width = get_ue_golomb(gb);
        height = get_ue_golomb(gb);
        /* pic conformance_flag */
        if (get_bits(gb, 1)) {
            int h_scale, v_scale;
            /* pads of subsampled dimensions were right shifted. undo. */
            h_scale = (chroma_format_idc  % 3) ? 2 : 1;
            v_scale = (chroma_format_idc == 1) ? 2 : 1;

            /* undo picture padding */
            width -= get_ue_golomb(gb) * h_scale; /* left_offset */
            width -= get_ue_golomb(gb) * h_scale; /* right_offset */
            height -= get_ue_golomb(gb) * v_scale; /* top_offset */
            height -= get_ue_golomb(gb) * v_scale; /* bottom_offset */
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
        sublayer_ordering_info = get_bits(gb, 1);
        /* should be 1|0|1 instead of 2|1|1 but x265 doesn't do that */
        if (   2 < get_ue_golomb(gb) /* max_dec_pic_buffering */
            || 1 < get_ue_golomb(gb) /* num_reorder_pics */
            || 1 < get_ue_golomb(gb) /* max_latency_increase, 0 is infinite */)
        {
            fprintf(stderr, "Stream should be marked as monotically ordered (zero-delay)\n");
            goto bad_header_info;
        }

        log2_min_cb_size = get_ue_golomb(gb) + 3;
        min_cb_size = 1 << log2_min_cb_size;
        log2_diff_max_min_coding_block_size = get_ue_golomb(gb);
        log2_min_tb_size = get_ue_golomb(gb) + 2;
        log2_diff_max_min_transform_block_size = get_ue_golomb(gb);

        max_transform_hierarchy_depth_inter = get_ue_golomb(gb);
        max_transform_hierarchy_depth_intra = get_ue_golomb(gb);
        if (max_transform_hierarchy_depth_inter != max_transform_hierarchy_depth_intra) {
            fprintf(stderr, "max_transform_hierarchy_depth_inter must be the same as max_transform_hierarchy_depth_intra (%d %d)\n", max_transform_hierarchy_depth_inter, max_transform_hierarchy_depth_intra);
            goto bad_header_info;
        }

        scaling_list_enable_flag = get_bits(gb, 1);
        if (scaling_list_enable_flag != 0) {
            fprintf(stderr, "scaling_list_enable_flag must be 0\n");
            return -1;
        }
        amp_enabled_flag = get_bits(gb, 1);
        if (!amp_enabled_flag) {
            fprintf(stderr, "amp_enabled_flag must be set\n");
            goto bad_header_info;
        }
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
        if (!sps_temporal_mvp_enabled_flag) {
            fprintf(stderr, "sps_temporal_mvp_enabled_flag must be set\n");
            goto bad_header_info;
        }
        sps_strong_intra_smoothing_enable_flag = get_bits(gb, 1);
        vui_present = get_bits(gb, 1);
        if (vui_present) {
            int sar_present, sar_idx, overscan_info_present_flag;
            int video_signal_type_present_flag, chroma_loc_info_present_flag;
            int default_display_window_flag, vui_timing_info_present_flag;
            int vui_poc_proportional_to_timing_flag;
            int vui_hrd_parameters_present_flag, bitstream_restriction_flag;
            int colour_description_present_flag;

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
                    int primaries, transfer, matrix;

                    primaries = get_bits(gb, 8);
                    transfer = get_bits(gb, 8); /* transfer_characteristics */
                    matrix = get_bits(gb, 8); /* matrix_coeffs */
                    if (primaries < 1 || primaries > 2) {
                        fprintf(stderr, "only sRGB/BT709 color-space is supported (%d)\n", primaries);
                        goto bad_header_info;
                    }
                    if (transfer != 2 && transfer != 13)
                        fprintf(stderr, "warning: sRGB gamma will be assumed\n");
                    if (matrix == 0) {
                        color_space = BPG_CS_RGB; // GBR, probably
                    } else if (matrix == 1 || matrix == 2) {
                        color_space = BPG_CS_YCbCr_BT709; // YCbCr BT709, probably
                    } else if (matrix == 4 || matrix == 5 || matrix == 6) {
                        color_space = BPG_CS_YCbCr; // YCbCr BT601, probably
                    } else if (matrix == 8) {
                        color_space = BPG_CS_YCgCo; // YCgCo
                    } else if (matrix == 9 || matrix == 10) {
                        color_space = matrix - 5; // YCbCr BT2020 NCL and CL
                    } else  {
                        fprintf(stderr, "unsupported color space (%d)\n", matrix);
                        goto bad_header_info;
                    }
                }
            }
            chroma_loc_info_present_flag = get_bits(gb, 1);
            if (chroma_loc_info_present_flag) {
                int loc_top, loc_bot;
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
    /* ret is used instead of idx because idx to store location of SPS's end */
    ret = extract_nal(&nal_buf, &nal_len, buf + idx, buf_len);
    if (ret < 0)
        goto bad_header_info;
    nal_unit_type = (nal_buf[0] >> 1) & 0x3f;
    if (nal_unit_type != 34) {
        fprintf(stderr, "expecting PPS nal, got (%d)\n", nal_unit_type);
        goto bad_header_info;
    }
    {
        int cu_qp_delta;

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

        cu_qp_delta = get_bits(gb, 1);
        if (cu_qp_delta)
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

    {
        HEVCVideoConfig *v;
        v = mallocz(sizeof(HEVCVideoConfig));
        if (!v)
            goto bad_header_info;

        /* store stream specific config */
        v->width = width;
        v->height = height;
        v->min_cb_size = min_cb_size;
        v->fps_num = fps_num;
        v->fps_den = fps_den;

        v->bit_depth = bit_depth;
        v->limited_range = !video_full_range_flag;
        v->pixel_format = pixel_format;
        v->color_space = color_space;

        v->parallel = tiles | (wpp << 1);
        v->dependent_slices = dependent_slices;

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

static int add_frame_duration_sei(DynBuf *out_buf, uint16_t frame_ticks)
{
    uint8_t nal_buf[128], *q;
    int nut, nal_len;

    q = nal_buf;
    *q++ = 0x00;
    *q++ = 0x00;
    *q++ = 0x01;
    nut = 39; /* prefix SEI NUT */
    *q++ = (nut << 1);
    *q++ = 1;
    *q++ = 0xff;  /* payload_type = 257 */
    *q++ = 0x02;
    *q++ = 2; /* payload_size = 2 */
    *q++ = frame_ticks >> 8;
    *q++ = frame_ticks;
    *q++ = 0x80; /* extra '1' bit and align to byte */
    /* Note: the 0x00 0x00 b pattern with b <= 3 cannot happen, so no
       need to escape */
    nal_len = q - nal_buf;
    if (dyn_buf_resize(out_buf, out_buf->len + nal_len) < 0)
        return -1;
    memcpy(out_buf->buf + out_buf->len, nal_buf, nal_len);
    out_buf->len += nal_len;
    return 0;
}

static int build_modified_hevc(uint8_t **pout_buf, BPGMuxerContext *s,
                               int cbuf_len, int abuf_len,
                               FILE *fdelay)
{
    DynBuf out_buf_s, *out_buf = &out_buf_s;
    const uint8_t *cbuf = s->color_buf;
    const uint8_t *abuf = s->alpha_buf;
    const uint8_t *nal_buf;

    int msps_len, cidx, aidx, is_alpha, nal_len, first_nal, start, l;
    int nut;

    dyn_buf_init(out_buf);

    /* add alpha MSPS */
    aidx = 0; /* avoids warning */
    if (abuf) {
        aidx = s->alpha.pps_idx;
        msps_len = s->alpha.msps_len;
        memcpy(out_buf->buf + out_buf->len, s->alpha.msps, msps_len);
        out_buf->len += msps_len;
        free(s->alpha.msps);
        s->alpha.msps = NULL;
    }

    /* add color MSPS */
    cidx = s->color.pps_idx;
    msps_len = s->color.msps_len;
    if (cidx < 0)
        goto fail;
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
               (parse_slice(nal_buf + start, nal_len, nut, &s->color) >= 0)) ) {
            fprintf(stderr, "error while parsing (alpha? %d) NALs\n", is_alpha);
            goto fail;
        }

        /* store animation data for displayed frames */
        if (!is_alpha && nut <= RSV_IRAP) {
            int frame_ticks;

            /* if delay file exists, use delay timings from there for each new frame */
            if (fdelay) {
                float fdelay_val;
                if (fscanf(fdelay, "%f", &fdelay_val) == 1) {
                    frame_ticks = lrint(fdelay_val * s->frame_delay_den / (s->frame_delay_num * 100));
                    if (frame_ticks < 1)
                        frame_ticks = 1;
                    if (frame_ticks >= 1 && frame_ticks <= 65535)
                        s->frame_ticks = frame_ticks;
                }
            }
            frame_ticks = s->frame_ticks;

            /* store the frame duration */
            if ((s->frame_count + 1) > s->frame_duration_tab_size) {
                int tab_sz = s->frame_duration_tab_size;

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
    return -1;
}

BPGMuxerContext *bpg_muxer_open()
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

    s->color_space = BPG_CS_COUNT;
    s->limited_range = -1;
    s->premultiplied_alpha = 0;
    s->cmyk = 0;
    return s;
}

static int my_write_func(void *opaque, const uint8_t *buf, int buf_len)
{
    FILE *f = opaque;
    return fwrite(buf, 1, buf_len, f);
}

static int bpg_muxer_finish_ext(BPGMuxerContext *s, void *opaque) {
    uint8_t *extension_buf;
    int extension_buf_len;

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
        int max_len;
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
        int v, has_alpha, has_extension, alpha2_flag, alpha1_flag, format;

        has_alpha = (s->alpha_buf != NULL);
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
            (alpha2_flag << 2) | (s->color.limited_range << 1) |
            (s->frame_count > 1);//
        *q++ = v;
        put_ue(&q, s->color.width);
        put_ue(&q, s->color.height);

        put_ue(&q, 0); /* zero length means up to the end of the file */
        if (has_extension) {
            put_ue(&q, extension_buf_len); /* extension data length */
        }

        my_write_func(opaque, img_header, q - img_header);

        if (has_extension) {
            if (my_write_func(opaque, extension_buf, extension_buf_len) != extension_buf_len) {
                fprintf(stderr, "Error while writing extension data\n");
                free(extension_buf);
                return -1;
            }
            free(extension_buf);
        }
    }

    return 0;
}

int bpg_muxer_load_hevc(uint8_t **pbuf, HEVCVideoConfig *cfg, const char *hevc_name)
{
    FILE *f;
    uint8_t *out_buf;
    int out_buf_len;

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
    return -1;
}

int check_hevc_cfg(BPGMuxerContext *s)
{
    const HEVCVideoConfig *color, *alpha;
    color = &s->color;
    alpha = &s->alpha;

    if (s->limited_range < 0) {
        s->limited_range = color->limited_range;
    }

    if (s->frame_delay_num <= 0 || s->frame_delay_den <= 0) {
        if (color->fps_num && color->fps_den
            && color->fps_num <= USHRT_MAX
            && color->fps_den <= USHRT_MAX)
        {
            /* delay is 1/fps */
            s->frame_delay_num = color->fps_den;
            s->frame_delay_den = color->fps_num;
        } else {
            s->frame_delay_num = 1;
            s->frame_delay_den = 25;
        }
    }

    if (s->alpha_buf) {
        if (alpha->width != color->width
            || alpha->height != color->height
            || alpha->bit_depth != color->bit_depth
            || alpha->parallel != color->parallel)
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
    }

    if (s->color_space == BPG_CS_COUNT) {
        s->color_space = color->color_space;
    }
    return 0;
}

void bpg_muxer_close(BPGMuxerContext *s)
{
    bpg_md_free(s->first_md);
    //if (s->color) {
        if (s->color.msps)
            free(s->color.msps);
    //    free(s->color);
    //}
    //if (s->alpha) {
        if (s->alpha.msps)
            free(s->alpha.msps);
    //    free(s->alpha);
    //}
    free(s->color_buf);
    free(s->alpha_buf);

    free(s->frame_duration_tab);
    free(s);
}

void help(int is_full)
{
    printf("BPG Muxer version " CONFIG_BPG_VERSION "\n"
           "usage: bpgmux [options] infile.[h265|hevc]\n"
           "\n"
           "Inputs streams should be monotonically ordered with\n"
           "at most one reference frame at minus one POC\n"
           "\n"
           "Main options:\n"
           "-h                   show the full help (including the advanced options)\n"
           "-a alpha_fname       name of 4th plane encoded input for muxing\n"
           "-o muxed_fname       set output filename (default = %s)\n"
           "-c color_space       set the input color space (ycbcr, rgb, ycgco,\n"
           "                         ycbcr_bt709, ycbcr_bt2020, default=ycbcr_bt709)\n"
           "\n"
           "Animation options:\n"
           "-fps N               set the frame rate (default = 25)\n"
           "-loop N              set the number of times the animation is played. 0 means\n"
           "                         infinite (default = 0)\n"
           "-delayfile file      text file containing one number per image giving the\n"
           "                         display delay per image in centiseconds.\n"
           , DEFAULT_OUTFILENAME);

    if (is_full) {
        printf("\nAdvanced options:\n"
           "-premul              set if input is already premultiplied with alpha (0 or 1, default 0)\n"
           "-limitedrange        manually indicate whether input has clamped video range (0 or 1, default 1)\n"
           "\n\n");
    }

    exit(1);
}

struct option long_opts[] = {
    { "limitedrange", no_argument },
    { "premul", no_argument },
    { "loop", required_argument },
    { "fps", required_argument },
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
    int color_buf_len, alpha_buf_len;
    int muxed_buf_len;

    BPGMuxerContext *s;
    s = bpg_muxer_open();

    muxed_fname = DEFAULT_OUTFILENAME;
    frame_delay_file = NULL;
    alpha_fname = NULL;
    md = NULL;
    muxed_buf = NULL;

    for(;;) {
        c = getopt_long_only(argc, argv, "a:o:c:h", long_opts, &option_index);
        if (c == -1)
            break;
        switch(c) {
        case 0:
            switch(option_index) {
            case 0:
                s->limited_range = 1;
                break;
            case 1:
                s->premultiplied_alpha = 1;
                break;
            case 2:
                s->loop_count = strtoul(optarg, NULL, 0);
                break;
            case 3:
                if (2 != sscanf(optarg, "%hu/%hu", &s->frame_delay_den, &s->frame_delay_num)) {
                    double fps = atof(optarg);
                    if (fps > 0 && fps <= USHRT_MAX/1000) {
                        s->frame_delay_num = 1000;
                        s->frame_delay_den = lrint(fps * 1000.0);
                    } else if (fps > 0 && fps <= USHRT_MAX/100) {
                        s->frame_delay_num = 100;
                        s->frame_delay_den = lrint(fps * 100.0);
                    } else if (fps > 0 && fps <= USHRT_MAX/10) {
                        s->frame_delay_num = 10;
                        s->frame_delay_den = lrint(fps * 10.0);
                    } else {
                        s->frame_delay_num = 1;
                        s->frame_delay_den = strtoul(optarg, NULL, 0);
                    }
                }
                if (s->frame_delay_den == 0) {
                    fprintf(stderr, "invalid frame rate\n");
                    exit(1);
                }
                break;
            case 4:
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
            help(1);
            break;
        default:
            exit(1);
        }
    }

    if (optind >= argc)
        help(0);
    color_fname = argv[optind];

    f = fopen(muxed_fname, "wb");
    if (!f) {
        perror(muxed_fname);
        goto bpgmux_fail;
    }

    /* load input bitstreams into memory. also check per-bistream conformity */
    color_buf_len = bpg_muxer_load_hevc(&s->color_buf, &s->color, color_fname);
    if (color_buf_len < 0)
        goto bpgmux_fail;
    if (alpha_fname) {
        alpha_buf_len = bpg_muxer_load_hevc(&s->alpha_buf, &s->alpha, alpha_fname);
        if (alpha_buf_len < 0)
            goto bpgmux_fail;
    }
    /* make sure muxer_ctx settings and color and alpha agree */
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
    muxed_buf_len = build_modified_hevc(&muxed_buf, s, color_buf_len, alpha_buf_len, fdelay);
    if (muxed_buf_len < 0) {
        fprintf(stderr, "Error while building modified hevc buffer\n");
        goto bpgmux_fail;
    }

    /* start the writing process */

    if (bpg_muxer_finish_ext(s, f) < 0) {
        fprintf(stderr, "Error while writing extension data to disk\n");
        goto bpgmux_fail;
    }

    if (my_write_func(f, muxed_buf, muxed_buf_len) != muxed_buf_len) {
        fprintf(stderr, "Error while writing HEVC data to disk\n");
        goto bpgmux_fail;
    }
    
    fclose(f);

    bpg_muxer_close(s);
    exit(0);

  bpgmux_fail:
    fprintf(stderr, "Muxing has failed.\n");
    if (muxed_buf)
        free (muxed_buf);
    bpg_muxer_close(s);
    exit(1);
}



