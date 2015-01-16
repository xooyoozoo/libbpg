/*
 * BPG encoder
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
#include <getopt.h>
#include <math.h>
#include <assert.h>

#include "bpgfmt.h"

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
int find_nal_end(const uint8_t *buf, int buf_len)
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
int extract_nal(uint8_t **pnal_buf, int *pnal_len,
                       const uint8_t *buf, const int buf_len)
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

void init_get_bits(GetBitState *s, const uint8_t *buf, int buf_len)
{
    s->buf = buf;
    s->buf_len = buf_len;
    s->idx = 0;
}

void skip_bits(GetBitState *s, int n)
{
    s->idx += n;
}

/* 1 <= n <= 25. return '0' bits if past the end of the buffer. */
uint32_t get_bits(GetBitState *s, int n)
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
uint32_t get_bits_long(GetBitState *s, int n)
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
uint32_t get_ue_golomb(GetBitState *s)
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

void init_put_bits(PutBitState *s, uint8_t *buf)
{
    s->buf = buf;
    s->idx = 0;
}

void put_bit(PutBitState *s, int bit)
{
    s->buf[s->idx >> 3] |= bit << (7 - (s->idx & 7));
    s->idx++;
}

void put_bits(PutBitState *s, int n, uint32_t v)
{
    int i;

    for(i = 0; i < n; i++) {
        put_bit(s, (v >> (n - 1 - i)) & 1);
    }
}

void put_ue_golomb(PutBitState *s, uint32_t v)
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

/* big endian variable length 7 bit encoding */
void put_ue(uint8_t **pp, uint32_t v)
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

void dyn_buf_init(DynBuf *s)
{
    s->buf = NULL;
    s->size = 0;
    s->len = 0;
}

int dyn_buf_resize(DynBuf *s, int size)
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

int add_frame_duration_sei(DynBuf *out_buf, uint16_t frame_ticks)
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

