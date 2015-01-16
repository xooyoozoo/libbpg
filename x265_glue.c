/*
 * x265 encoder front-end
 *
 * Copyright (c) 2014 Fabrice Bellard
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>
#include <unistd.h>

#include "bpgenc.h"

#include "x265.h"

struct HEVCEncoderContext {
    x265_encoder *enc;
    x265_picture *pic;
    uint8_t *buf;
    int buf_len, buf_size;
};

static HEVCEncoderContext *x265_open(const HEVCEncodeParams *params)
{
    HEVCEncoderContext *s;
    x265_param *p;
    int preset_index;
    const char *preset;

    s = malloc(sizeof(HEVCEncoderContext));
    memset(s, 0, sizeof(*s));

    if (params->bit_depth != x265_max_bit_depth) {
        fprintf(stderr, "x265 is compiled to support only %d bit depth. Use the '-b %d' option to force the bit depth.\n",
                x265_max_bit_depth, x265_max_bit_depth);
        return NULL;
    }
    if (params->chroma_format == BPG_FORMAT_GRAY) {
        fprintf(stderr, "x265 does not support monochrome (or alpha) data yet. Please use the jctvc encoder.\n");
        return NULL;
    }

    p = x265_param_alloc();

    preset_index = params->compress_level; /* 9 is placebo */

    preset = x265_preset_names[preset_index];
    if (params->verbose)
        printf("Using x265 preset: %s\n", preset);

    x265_param_default_preset(p, preset, "ssim");

    p->bRepeatHeaders = 1;
    p->decodedPictureHashSEI = params->sei_decoded_picture_hash;
    p->sourceWidth = params->width;
    p->sourceHeight = params->height;

    /* allow with tiny images and CTU size through log2(min_dimension) */
    switch ( (int)(log(params->width < params->height ? params->width : params->height)/log(2)) ) {
    case 4:
        p->maxCUSize = 16;
        p->tuQTMaxIntraDepth = 3;
        p->tuQTMaxInterDepth = 3;
        break;
    case 5:
        p->maxCUSize = 32;
        p->tuQTMaxIntraDepth = 4;
        p->tuQTMaxInterDepth = 4;
        break;
    default:
        break;
    }

    switch(params->chroma_format) {
    case BPG_FORMAT_GRAY:
        p->internalCsp = X265_CSP_I400;
        break;
    case BPG_FORMAT_420:
        p->internalCsp = X265_CSP_I420;
        break;
    case BPG_FORMAT_422:
        p->internalCsp = X265_CSP_I422;
        break;
    case BPG_FORMAT_444:
        p->internalCsp = X265_CSP_I444;
        break;
    default:
        abort();
    }
    if (params->intra_only) {
        p->keyframeMax = 1; /* only I frames */
        p->totalFrames = 1;
        p->rc.cuTree = 0;
    } else {
        p->keyframeMax = 250;
        p->totalFrames = 0;
        p->maxNumReferences = 1;
        p->bframes = 0;
    }
    p->bEnableRectInter = 1;
    p->bEnableAMP = 1; /* cannot use 0 due to header restriction */
    p->internalBitDepth = params->bit_depth;
    p->bEmitInfoSEI = 0;
    if (params->verbose) {
        p->bEnablePsnr = 1;
        p->logLevel = X265_LOG_INFO;
    } else
        p->logLevel = X265_LOG_NONE;

    /* dummy frame rate */
    p->fpsNum = 25;
    p->fpsDenom = 1;

    p->psyRd = params->psyrd;
    p->psyRdoq = params->psyrdoq;
    p->deblockingFilterBetaOffset = params->deblock;
    p->deblockingFilterTCOffset = params->deblock;
    /* sometimes, psy seems to cause blatant chroma noise */
    p->cbQpOffset = params->chroma_offset - (int)(0.5 + p->psyRd);
    p->crQpOffset = p->cbQpOffset;

    p->rc.rateControlMode = X265_RC_CRF;
    p->rc.rfConstant = params->qp;
    p->rc.aqMode = X265_AQ_VARIANCE;

    p->bEnableSAO = 1;

    p->bEnableWavefront = params->wpp;
    p->bLossless = params->lossless;

    s->enc = x265_encoder_open(p);

    s->pic = x265_picture_alloc();
    x265_picture_init(p, s->pic);

    s->pic->colorSpace = p->internalCsp;

    x265_param_free(p);

    return s;
}

static void add_nal(HEVCEncoderContext *s, const uint8_t *data, int data_len)
{
    int new_size, size;

    size = s->buf_len + data_len;
    if (size > s->buf_size) {
        new_size = (s->buf_size * 3) / 2;
        if (new_size < size)
            new_size = size;
        s->buf = realloc(s->buf, new_size);
        s->buf_size = new_size;
    }
    memcpy(s->buf + s->buf_len, data, data_len);
    s->buf_len += data_len;
}

static int x265_encode(HEVCEncoderContext *s, Image *img)
{
    int c_count, i, ret;
    x265_picture *pic;
    uint32_t nal_count;
    x265_nal *p_nal;

    pic = s->pic;

    if (img->format == BPG_FORMAT_GRAY)
        c_count = 1;
    else
        c_count = 3;
    for(i = 0; i < c_count; i++) {
        pic->planes[i] = img->data[i];
        pic->stride[i] = img->linesize[i];
    }
    pic->bitDepth = img->bit_depth;

    ret = x265_encoder_encode(s->enc, &p_nal, &nal_count, pic, NULL);
    if (ret > 0) {
        for(i = 0; i < nal_count; i++) {
            add_nal(s, p_nal[i].payload, p_nal[i].sizeBytes);
        }
    }
    return 0;
}

static int x265_close(HEVCEncoderContext *s, uint8_t **pbuf)
{
    int buf_len, ret, i;
    uint32_t nal_count;
    x265_nal *p_nal;

    /* get last compressed pictures */
    for(;;) {
        ret = x265_encoder_encode(s->enc, &p_nal, &nal_count, NULL, NULL);
        if (ret <= 0)
            break;
        for(i = 0; i < nal_count; i++) {
            /* only allow expected NAL types */
            if (p_nal[i].type == 40 || (p_nal[i].type / NAL_UNIT_ACCESS_UNIT_DELIMITER) < 1)
                add_nal(s, p_nal[i].payload, p_nal[i].sizeBytes);
        }
    }

    if (s->buf_len < s->buf_size) {
        s->buf = realloc(s->buf, s->buf_len);
    }

    *pbuf = s->buf;
    buf_len = s->buf_len;

    x265_encoder_close(s->enc);
    x265_picture_free(s->pic);
    free(s);
    return buf_len;
}

HEVCEncoder x265_hevc_encoder = {
  .open = x265_open,
  .encode = x265_encode,
  .close = x265_close,
};
