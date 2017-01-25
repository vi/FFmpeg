/*
 * Copyright (c) 2017 Vitaly "_Vi" Shukela
 * Copyright (c) 2015 Paul B Mahol
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * Simple temporal median denoiser for video with heavy, but transient glitches.
 * On each plane individually, it sorts values of pixel across last N frames
 * and chooses the middle one.
 * Not recommended for Y plane.
 * Based on vt_atadenoise.c
 */

#include "libavutil/imgutils.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "avfilter.h"

#define FF_BUFQUEUE_SIZE 129
#include "bufferqueue.h"

#include "formats.h"
#include "internal.h"
#include "video.h"

#define SIZE FF_BUFQUEUE_SIZE

typedef struct MedianDenoiseContext {
    const AVClass *class;

    int planes;
    int nb_planes;
    int planewidth[4];
    int planeheight[4];

    struct FFBufQueue q;
    void *data[4][SIZE];
    int linesize[4][SIZE];
    int size, mid;
    int available;

    int (*filter_slice)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs);
} MedianDenoiseContext;

#define OFFSET(x) offsetof(MedianDenoiseContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

static const AVOption mediandenoise_options[] = {
    { "s",  "set how many frames to use",    OFFSET(size),     AV_OPT_TYPE_INT,   {.i64=5},    3, SIZE, FLAGS },
    { "p",  "set what planes to filter",     OFFSET(planes),   AV_OPT_TYPE_FLAGS, {.i64=6},    0, 15,  FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(mediandenoise);

static int query_formats(AVFilterContext *ctx)
{
    static const enum AVPixelFormat pixel_fmts[] = {
        AV_PIX_FMT_GRAY8,
        AV_PIX_FMT_GRAY16,
        AV_PIX_FMT_YUV410P, AV_PIX_FMT_YUV411P,
        AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUV422P,
        AV_PIX_FMT_YUV440P, AV_PIX_FMT_YUV444P,
        AV_PIX_FMT_YUVJ420P, AV_PIX_FMT_YUVJ422P,
        AV_PIX_FMT_YUVJ440P, AV_PIX_FMT_YUVJ444P,
        AV_PIX_FMT_YUVJ411P,
        AV_PIX_FMT_YUV420P9, AV_PIX_FMT_YUV422P9, AV_PIX_FMT_YUV444P9,
        AV_PIX_FMT_YUV420P10, AV_PIX_FMT_YUV422P10, AV_PIX_FMT_YUV444P10,
        AV_PIX_FMT_YUV440P10,
        AV_PIX_FMT_YUV444P12, AV_PIX_FMT_YUV422P12, AV_PIX_FMT_YUV420P12,
        AV_PIX_FMT_YUV440P12,
        AV_PIX_FMT_YUV444P14, AV_PIX_FMT_YUV422P14, AV_PIX_FMT_YUV420P14,
        AV_PIX_FMT_YUV420P16, AV_PIX_FMT_YUV422P16, AV_PIX_FMT_YUV444P16,
        AV_PIX_FMT_GBRP, AV_PIX_FMT_GBRP9, AV_PIX_FMT_GBRP10,
        AV_PIX_FMT_GBRP12, AV_PIX_FMT_GBRP14, AV_PIX_FMT_GBRP16,
        AV_PIX_FMT_NONE
    };
    AVFilterFormats *formats = ff_make_format_list(pixel_fmts);
    if (!formats)
        return AVERROR(ENOMEM);
    return ff_set_common_formats(ctx, formats);
}

static av_cold int init(AVFilterContext *ctx)
{
    MedianDenoiseContext *s = ctx->priv;

    if (!(s->size & 1)) {
        av_log(ctx, AV_LOG_ERROR, "size %d is invalid. Must be an odd value.\n", s->size);
        return AVERROR(EINVAL);
    }
    s->mid = s->size / 2 + 1;

    return 0;
}

static int cmp_int (const void * a, const void * b)
{
   return ( ((int)*(uint8_t*)a) - ((int)*(uint8_t*)b) );
}


typedef struct ThreadData {
    AVFrame *in, *out;
} ThreadData;

static int filter_slice8(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    MedianDenoiseContext *s = ctx->priv;
    ThreadData *td = arg;
    AVFrame *in = td->in;
    AVFrame *out = td->out;
    const int size = s->size;
    const int mid = s->mid;
    int p, x, y, i;

    int sortbuf[SIZE];

    for (p = 0; p < s->nb_planes; p++) {
        const int h = s->planeheight[p];
        const int w = s->planewidth[p];
        const int slice_start = (h * jobnr) / nb_jobs;
        const int slice_end = (h * (jobnr+1)) / nb_jobs;
        const uint8_t *src = in->data[p] + slice_start * in->linesize[p];
        uint8_t *dst = out->data[p] + slice_start * out->linesize[p];
        const uint8_t **data = (const uint8_t **)s->data[p];
        const int *linesize = (const int *)s->linesize[p];
        const uint8_t *srcf[SIZE];

        if (!((1 << p) & s->planes)) {
            av_image_copy_plane(dst, out->linesize[p], src, in->linesize[p],
                                w, slice_end - slice_start);
            continue;
        }

        for (i = 0; i < size; i++)
            srcf[i] = data[i] + slice_start * linesize[i];

        for (y = slice_start; y < slice_end; y++) {
            for (x = 0; x < w; x++) {

                for (i = 0; i < size; i++) {
                    sortbuf[i] = srcf[i][x];
                }

                qsort(sortbuf, size, sizeof(sortbuf[0]), cmp_int);

                dst[x] = sortbuf[mid];
            }

            dst += out->linesize[p];
            src += in->linesize[p];

            for (i = 0; i < size; i++)
                srcf[i] += linesize[i];
        }
    }

    return 0;
}

static int filter_slice16(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    MedianDenoiseContext *s = ctx->priv;
    ThreadData *td = arg;
    AVFrame *in = td->in;
    AVFrame *out = td->out;
    const int size = s->size;
    const int mid = s->mid;
    int p, x, y, i;

    int sortbuf[SIZE];

    for (p = 0; p < s->nb_planes; p++) {
        const int h = s->planeheight[p];
        const int w = s->planewidth[p];
        const int slice_start = (h * jobnr) / nb_jobs;
        const int slice_end = (h * (jobnr+1)) / nb_jobs;
        const uint16_t *src = (uint16_t *)(in->data[p] + slice_start * in->linesize[p]);
        uint16_t *dst = (uint16_t *)(out->data[p] + slice_start * out->linesize[p]);
        const uint8_t **data = (const uint8_t **)s->data[p];
        const int *linesize = (const int *)s->linesize[p];
        const uint16_t *srcf[SIZE];

        if (!((1 << p) & s->planes)) {
            av_image_copy_plane((uint8_t *)dst, out->linesize[p], (uint8_t *)src, in->linesize[p],
                                w * 2, slice_end - slice_start);
            continue;
        }

        for (i = 0; i < s->size; i++)
            srcf[i] = (const uint16_t *)(data[i] + slice_start * linesize[i]);

        for (y = slice_start; y < slice_end; y++) {
            for (x = 0; x < w; x++) {
                for (i = 0; i < size; i++) {
                    sortbuf[i] = srcf[i][x];
                }

                qsort(sortbuf, size, sizeof(sortbuf[0]), cmp_int);

                dst[x] = sortbuf[mid];
            }

            dst += out->linesize[p] / 2;
            src += in->linesize[p] / 2;

            for (i = 0; i < size; i++)
                srcf[i] += linesize[i] / 2;
        }
    }

    return 0;
}

static int config_input(AVFilterLink *inlink)
{
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);
    AVFilterContext *ctx = inlink->dst;
    MedianDenoiseContext *s = ctx->priv;
    int depth;

    s->nb_planes = desc->nb_components;

    s->planeheight[1] = s->planeheight[2] = AV_CEIL_RSHIFT(inlink->h, desc->log2_chroma_h);
    s->planeheight[0] = s->planeheight[3] = inlink->h;
    s->planewidth[1]  = s->planewidth[2]  = AV_CEIL_RSHIFT(inlink->w, desc->log2_chroma_w);
    s->planewidth[0]  = s->planewidth[3]  = inlink->w;

    depth = desc->comp[0].depth;
    if (depth == 8)
        s->filter_slice = filter_slice8;
    else
        s->filter_slice = filter_slice16;

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *buf)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    MedianDenoiseContext *s = ctx->priv;
    AVFrame *out, *in;
    int i;

    if (s->q.available != s->size) {
        if (s->q.available < s->mid) {
            for (i = 0; i < s->mid; i++) {
                out = av_frame_clone(buf);
                if (!out) {
                    av_frame_free(&buf);
                    return AVERROR(ENOMEM);
                }
                ff_bufqueue_add(ctx, &s->q, out);
            }
        }
        if (s->q.available < s->size) {
            ff_bufqueue_add(ctx, &s->q, buf);
            s->available++;
        }
        return 0;
    }

    in = ff_bufqueue_peek(&s->q, s->mid);

    if (!ctx->is_disabled) {
        ThreadData td;

        out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
        if (!out) {
            av_frame_free(&buf);
            return AVERROR(ENOMEM);
        }

        for (i = 0; i < s->size; i++) {
            AVFrame *frame = ff_bufqueue_peek(&s->q, i);

            s->data[0][i] = frame->data[0];
            s->data[1][i] = frame->data[1];
            s->data[2][i] = frame->data[2];
            s->linesize[0][i] = frame->linesize[0];
            s->linesize[1][i] = frame->linesize[1];
            s->linesize[2][i] = frame->linesize[2];
        }

        td.in = in; td.out = out;
        ctx->internal->execute(ctx, s->filter_slice, &td, NULL,
                               FFMIN3(s->planeheight[1],
                                      s->planeheight[2],
                                      ff_filter_get_nb_threads(ctx)));
        av_frame_copy_props(out, in);
    } else {
        out = av_frame_clone(in);
        if (!out) {
            av_frame_free(&buf);
            return AVERROR(ENOMEM);
        }
    }

    in = ff_bufqueue_get(&s->q);
    av_frame_free(&in);
    ff_bufqueue_add(ctx, &s->q, buf);

    return ff_filter_frame(outlink, out);
}

static int request_frame(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    MedianDenoiseContext *s = ctx->priv;
    int ret = 0;

    ret = ff_request_frame(ctx->inputs[0]);

    if (ret == AVERROR_EOF && !ctx->is_disabled && s->available) {
        AVFrame *buf = av_frame_clone(ff_bufqueue_peek(&s->q, s->available));
        if (!buf)
            return AVERROR(ENOMEM);

        ret = filter_frame(ctx->inputs[0], buf);
        s->available--;
    }

    return ret;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    MedianDenoiseContext *s = ctx->priv;

    ff_bufqueue_discard_all(&s->q);
}

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = filter_frame,
        .config_props = config_input,
    },
    { NULL }
};

static const AVFilterPad outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_VIDEO,
        .request_frame = request_frame,
    },
    { NULL }
};

AVFilter ff_vf_mediandenoise = {
    .name          = "mediandenoise",
    .description   = NULL_IF_CONFIG_SMALL("Apply an Median Temporal Denoiser."),
    .priv_size     = sizeof(MedianDenoiseContext),
    .priv_class    = &mediandenoise_class,
    .init          = init,
    .uninit        = uninit,
    .query_formats = query_formats,
    .inputs        = inputs,
    .outputs       = outputs,
    .flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL | AVFILTER_FLAG_SLICE_THREADS,
};


