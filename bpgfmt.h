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
#ifdef __cplusplus
extern "C" {
#endif

#include "libbpg.h"

extern void *mallocz(size_t size);

extern int find_nal_end(const uint8_t *buf, int buf_len);

typedef struct BPGMetaData {
    uint32_t tag;
    uint8_t *buf;
    int buf_len;
    struct BPGMetaData *next;
} BPGMetaData;

extern BPGMetaData *bpg_md_alloc(uint32_t tag);
extern void bpg_md_free(BPGMetaData *md);

typedef struct {
    const uint8_t *buf;
    int idx;
    int buf_len;
} GetBitState;

extern void init_get_bits(GetBitState *s, const uint8_t *buf, int buf_len);
extern void skip_bits(GetBitState *s, int n);
extern uint32_t get_bits(GetBitState *s, int n);
extern uint32_t get_bits_long(GetBitState *s, int n);
extern uint32_t get_ue_golomb(GetBitState *s);

typedef struct {
    uint8_t *buf;
    int idx;
} PutBitState;

extern void init_put_bits(PutBitState *s, uint8_t *buf);
extern void put_bit(PutBitState *s, int bit);
extern void put_bits(PutBitState *s, int n, uint32_t v);
extern void put_ue_golomb(PutBitState *s, uint32_t v);
extern void put_ue(uint8_t **pp, uint32_t v);

typedef struct {
    uint8_t *buf;
    int size;
    int len;
} DynBuf;

extern void dyn_buf_init(DynBuf *s);
extern int dyn_buf_resize(DynBuf *s, int size);

extern int add_frame_duration_sei(DynBuf *out_buf, uint16_t frame_ticks);


#ifdef __cplusplus
}
#endif

