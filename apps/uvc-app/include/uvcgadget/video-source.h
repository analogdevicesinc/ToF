/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Abstract video source
 *
 * Copyright (C) 2018 Laurent Pinchart
 *
 * Contact: Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 */
#ifndef __VIDEO_SOURCE_H__
#define __VIDEO_SOURCE_H__

struct v4l2_buffer;
struct v4l2_pix_format;
struct video_buffer;
struct video_buffer_set;
struct video_source;

struct video_source_ops {
	void(*destroy)(struct video_source *src);
	int(*set_format)(struct video_source *src, struct v4l2_pix_format *fmt);
	int(*set_frame_rate)(struct video_source *src, unsigned int fps);
	int(*alloc_buffers)(struct video_source *src, unsigned int nbufs);
	int(*export_buffers)(struct video_source *src,
			     struct video_buffer_set **buffers);
	int(*free_buffers)(struct video_source *src);
	int(*stream_on)(struct video_source *src);
	int(*stream_off)(struct video_source *src);
	int(*queue_buffer)(struct video_source *src, struct video_buffer *buf);
};

typedef void(*video_source_buffer_handler_t)(void *, struct video_source *,
					     struct video_buffer *);

struct video_source {
	const struct video_source_ops *ops;
	struct events *events;
	video_source_buffer_handler_t handler;
	void *handler_data;
};

void video_source_set_buffer_handler(struct video_source *src,
				     video_source_buffer_handler_t handler,
				     void *data);
void video_source_destroy(struct video_source *src);
int video_source_set_format(struct video_source *src,
			    struct v4l2_pix_format *fmt);
int video_source_set_frame_rate(struct video_source *src, unsigned int fps);
int video_source_alloc_buffers(struct video_source *src, unsigned int nbufs);
int video_source_export_buffers(struct video_source *src,
				struct video_buffer_set **buffers);
int video_source_free_buffers(struct video_source *src);
int video_source_stream_on(struct video_source *src);
int video_source_stream_off(struct video_source *src);
int video_source_queue_buffer(struct video_source *src,
			      struct video_buffer *buf);

#endif /* __VIDEO_SOURCE_H__ */
