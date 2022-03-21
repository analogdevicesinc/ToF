/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * UVC protocol handling
 *
 * Copyright (C) 2010-2018 Laurent Pinchart
 *
 * Contact: Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 */

#include <errno.h>
#include <limits.h>
#include <linux/usb/ch9.h>
#include <linux/usb/g_uvc.h>
#include <linux/usb/video.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>

#include "configfs.h"
#include "events.h"
#include "stream.h"
#include "tools.h"
#include "uvc.h"
#include "v4l2.h"
#include "../tof-sdk-interface.h"

#define ENABLE_USB_REQUEST_DEBUG
#define MAX_PACKET_SIZE 60

#ifdef ENABLE_USB_REQUEST_DEBUG
#define USB_REQ_DEBUG(X...)                                                    \
    {                                                                          \
        printf("->%s: ", __FUNCTION__);                                        \
        printf(X);                                                             \
    }
#else
#define USB_REQ_DEBUG(X...)
#endif

struct uvc_data {
    char *out_buf;

    char *in_buf;
};

struct uvc_device {
    struct v4l2_device *vdev;

    struct uvc_stream *stream;
    struct uvc_function_config *fc;

    struct uvc_streaming_control probe;
    struct uvc_streaming_control commit;

    struct uvc_data data;

    int control;

    unsigned int fcc;
    unsigned int width;
    unsigned int height;
    unsigned int maxsize;
};

struct uvc_device *uvc_open(const char *devname, struct uvc_stream *stream) {
    struct uvc_device *dev;

    dev = malloc(sizeof *dev);
    if (dev == NULL)
        return NULL;

    memset(dev, 0, sizeof *dev);
    dev->stream = stream;

    dev->vdev = v4l2_open(devname);
    if (dev->vdev == NULL) {
        free(dev);
        return NULL;
    }

    return dev;
}

void uvc_close(struct uvc_device *dev) {
    v4l2_close(dev->vdev);
    dev->vdev = NULL;

    free(dev);
}

/* ---------------------------------------------------------------------------
 * Request processing
 */

static void uvc_fill_streaming_control(struct uvc_device *dev,
                                       struct uvc_streaming_control *ctrl,
                                       int iformat, int iframe,
                                       unsigned int ival) {
    const struct uvc_function_config_format *format;
    const struct uvc_function_config_frame *frame;
    unsigned int i;

    /*
	 * Restrict the iformat, iframe and ival to valid values. Negative
	 * values for iformat or iframe will result in the maximum valid value
	 * being selected.
	 */
    iformat = clamp((unsigned int)iformat, 1U, dev->fc->streaming.num_formats);
    format = &dev->fc->streaming.formats[iformat - 1];

    iframe = clamp((unsigned int)iframe, 1U, format->num_frames);
    frame = &format->frames[iframe - 1];

    for (i = 0; i < frame->num_intervals; ++i) {
        if (ival <= frame->intervals[i]) {
            ival = frame->intervals[i];
            break;
        }
    }

    if (i == frame->num_intervals)
        ival = frame->intervals[frame->num_intervals - 1];

    memset(ctrl, 0, sizeof *ctrl);

    ctrl->bmHint = 1;
    ctrl->bFormatIndex = iformat;
    ctrl->bFrameIndex = iframe;
    ctrl->dwFrameInterval = ival;

    switch (format->fcc) {
    case V4L2_PIX_FMT_YUYV:
    case V4L2_PIX_FMT_SBGGR16:
        ctrl->dwMaxVideoFrameSize = frame->width * frame->height * 2;
        break;
    case V4L2_PIX_FMT_SBGGR8:
        ctrl->dwMaxVideoFrameSize = frame->width * frame->height;
        break;
    case V4L2_PIX_FMT_MJPEG:
        ctrl->dwMaxVideoFrameSize = dev->maxsize;
        break;
    }

    ctrl->dwMaxPayloadTransferSize = dev->fc->streaming.ep.wMaxPacketSize;
    ctrl->bmFramingInfo = 3;
    ctrl->bPreferedVersion = 1;
    ctrl->bMaxVersion = 1;
}

static void uvc_events_process_standard(struct uvc_device *dev,
                                        const struct usb_ctrlrequest *ctrl,
                                        struct uvc_request_data *resp) {
    USB_REQ_DEBUG("standard request\n");
    (void)dev;
    (void)ctrl;
    (void)resp;
}

static void uvc_events_process_control(struct uvc_device *dev, uint8_t req,
                                       uint8_t cs, uint16_t len,
                                       struct uvc_request_data *resp) {
    USB_REQ_DEBUG("control request (req %02x cs %02x)\n", req, cs);
    (void)dev;
    (void)len;
    (void)resp;
}

static void uvc_events_process_extension_unit(struct uvc_device *dev,
                                              uint8_t req, uint8_t cs,
                                              uint16_t len,
                                              struct uvc_request_data *resp) {
    static int server_resp_bytes_sent;
    static size_t out_buf_len;
    static bool resp_len_sent = false;

    USB_REQ_DEBUG("extension unit request (req %02x cs %02x)\n", req, cs);

    switch (cs) {
    case 1: /* Client request */
        switch (req) {
        case UVC_SET_CUR:
            USB_REQ_DEBUG("Received SET_CUR on %d\n", cs);
            dev->control = 0x0301;
            resp->length = len;
            break;
        case UVC_GET_INFO:
            USB_REQ_DEBUG("Received GET_INFO on %d\n", cs);
            resp->data[0] = 0x02;
            resp->length = 1;
            break;
        case UVC_GET_LEN:
            USB_REQ_DEBUG("Received GET_LEN on %d\n", cs);
            resp->data[0] = MAX_PACKET_SIZE; // 60 bytes
            resp->data[1] = 0x00;
            resp->length = 2;
            USB_REQ_DEBUG("Responding with size %x for cs: %d\n", resp->data[0],
                          cs);
            break;
        case UVC_GET_MIN:
        case UVC_GET_MAX:
        case UVC_GET_DEF:
        case UVC_GET_RES:
            USB_REQ_DEBUG("Received %x on %d\n", req, cs);
            resp->data[0] = 0xff;
            resp->length = 1;
            break;
        default:
            printf("Unsupported bRequest: Received bRequest %x on cs %d\n", req,
                   cs);
            /*
            * We don't support this control, so STALL the
            * default control ep.
            */
            resp->length = -EL2HLT;
            break;
        }
        break;
    case 2: /* Server response */
        switch (req) {
        case UVC_GET_CUR:
            USB_REQ_DEBUG("Received GET_CUR on %d\n", cs);
            if (dev->data.out_buf) {
                if (resp_len_sent) {
                    size_t remaining_bytes_to_send =
                        (out_buf_len - server_resp_bytes_sent >
                         MAX_PACKET_SIZE)
                            ? MAX_PACKET_SIZE
                            : out_buf_len - server_resp_bytes_sent;
                    memcpy(resp->data,
                           dev->data.out_buf + server_resp_bytes_sent,
                           remaining_bytes_to_send);
                    server_resp_bytes_sent += remaining_bytes_to_send;
                    if (server_resp_bytes_sent == out_buf_len) {
                        // We're sent the entire response, now reset things to default
			free(dev->data.out_buf);
                        dev->data.out_buf = NULL;
                        resp_len_sent = false;
                    }
                    resp->length = remaining_bytes_to_send;
                } else {
		    USB_REQ_DEBUG("Out data len: %d\n", strlen(dev->data.out_buf));
                    out_buf_len = strlen(dev->data.out_buf);
                    memcpy(resp->data, (uint8_t *)&out_buf_len, sizeof(out_buf_len));
                    resp->length = out_buf_len;
                    resp_len_sent = true;
                    server_resp_bytes_sent = 0;
                }
            } else {
                resp->data[0] = 0x0;
                resp->length = 1;
            }
            break;
        case UVC_GET_INFO:
            USB_REQ_DEBUG("Received GET_INFO on %d\n", cs);
            resp->data[0] = 0x01;
            resp->length = 1;
            break;
        case UVC_GET_LEN:
            USB_REQ_DEBUG("Received GET_LEN on %d\n", cs);
            resp->data[0] = MAX_PACKET_SIZE; // 60 bytes
            resp->data[1] = 0x00;
            resp->length = 2;
            USB_REQ_DEBUG("Responding with size %x for cs: %d\n", resp->data[0],
                          cs);
            break;
        case UVC_GET_MIN:
        case UVC_GET_MAX:
        case UVC_GET_DEF:
        case UVC_GET_RES:
            USB_REQ_DEBUG("Received %x on %d\n", req, cs);
            resp->data[0] = 0xff;
            resp->length = 1;
            break;
        default:
            printf("Unsupported bRequest: Received bRequest %x on cs %d\n", req,
                   cs);
            /*
            * We don't support this control, so STALL the
            * default control ep.
            */
            resp->length = -EL2HLT;
            break;
        }
        break;
    default:
        printf("Unsupported cs: Received cs %d \n", cs);
        /*
        * We don't support this control, so STALL the control
        * ep.
        */
        resp->length = -EL2HLT;
        break;
    }
}

static void uvc_events_process_streaming(struct uvc_device *dev, uint8_t req,
                                         uint8_t cs,
                                         struct uvc_request_data *resp) {
    struct uvc_streaming_control *ctrl;

    USB_REQ_DEBUG("streaming request (req %02x cs %02x)\n", req, cs);

    if (cs != UVC_VS_PROBE_CONTROL && cs != UVC_VS_COMMIT_CONTROL)
        return;

    ctrl = (struct uvc_streaming_control *)&resp->data;
    resp->length = sizeof *ctrl;

    switch (req) {
    case UVC_SET_CUR:
        dev->control = cs;
        resp->length = 34;
        break;

    case UVC_GET_CUR:
        if (cs == UVC_VS_PROBE_CONTROL)
            memcpy(ctrl, &dev->probe, sizeof *ctrl);
        else
            memcpy(ctrl, &dev->commit, sizeof *ctrl);
        break;

    case UVC_GET_MIN:
    case UVC_GET_MAX:
    case UVC_GET_DEF:
        if (req == UVC_GET_MAX)
            uvc_fill_streaming_control(dev, ctrl, -1, -1, UINT_MAX);
        else
            uvc_fill_streaming_control(dev, ctrl, 1, 1, 0);
        break;

    case UVC_GET_RES:
        memset(ctrl, 0, sizeof *ctrl);
        break;

    case UVC_GET_LEN:
        resp->data[0] = 0x00;
        resp->data[1] = 0x22;
        resp->length = 2;
        break;

    case UVC_GET_INFO:
        resp->data[0] = 0x03;
        resp->length = 1;
        break;
    }
}

static void uvc_events_process_class(struct uvc_device *dev,
                                     const struct usb_ctrlrequest *ctrl,
                                     struct uvc_request_data *resp) {
    uint16_t interface = ctrl->wIndex;

    if ((ctrl->bRequestType & USB_RECIP_MASK) != USB_RECIP_INTERFACE)
        return;

    if (interface == dev->fc->control.intf.bInterfaceNumber)
        uvc_events_process_control(dev, ctrl->bRequest, ctrl->wValue >> 8,
                                   ctrl->wLength, resp);
    else if (interface == dev->fc->streaming.intf.bInterfaceNumber)
        uvc_events_process_streaming(dev, ctrl->bRequest, ctrl->wValue >> 8,
                                     resp);
    else if (ctrl->wIndex & 0x0300)
        uvc_events_process_extension_unit(
            dev, ctrl->bRequest, ctrl->wValue >> 8, ctrl->wLength, resp);
}

static void uvc_events_process_setup(struct uvc_device *dev,
                                     const struct usb_ctrlrequest *ctrl,
                                     struct uvc_request_data *resp) {
    dev->control = 0;

    USB_REQ_DEBUG("bRequestType %02x bRequest %02x wValue %04x wIndex %04x "
                  "wLength %04x\n",
                  ctrl->bRequestType, ctrl->bRequest, ctrl->wValue,
                  ctrl->wIndex, ctrl->wLength);

    switch (ctrl->bRequestType & USB_TYPE_MASK) {
    case USB_TYPE_STANDARD:
        uvc_events_process_standard(dev, ctrl, resp);
        break;

    case USB_TYPE_CLASS:
        uvc_events_process_class(dev, ctrl, resp);
        break;

    default:
        break;
    }
}

static void uvc_events_process_data(struct uvc_device *dev,
                                    const struct uvc_request_data *data) {
    const struct uvc_streaming_control *ctrl =
        (const struct uvc_streaming_control *)&data->data;
    struct uvc_streaming_control *target;
    static size_t client_req_bytes_read, in_buf_len;
    static bool blob_len_read = false;

    switch (dev->control) {
    case UVC_VS_PROBE_CONTROL:
        USB_REQ_DEBUG("setting probe control, length = %d\n", data->length);
        target = &dev->probe;
        break;

    case UVC_VS_COMMIT_CONTROL:
        USB_REQ_DEBUG("setting commit control, length = %d\n", data->length);
        target = &dev->commit;
        break;

    case 0x0301: //Extension unit ID 3, CS 1
        USB_REQ_DEBUG("getting extension unit, length = %d\n", data->length);
        //Store the client request string sent from remote
        if (blob_len_read) {
            // Read one packet and append
            size_t remaining_bytes_to_read =
                in_buf_len - client_req_bytes_read;
            size_t packet_size = (remaining_bytes_to_read > MAX_PACKET_SIZE)
                                     ? MAX_PACKET_SIZE
                                     : remaining_bytes_to_read;
            memcpy(dev->data.in_buf + client_req_bytes_read, data->data,
                   packet_size);
            client_req_bytes_read += packet_size;
            remaining_bytes_to_read -= packet_size;

            if (remaining_bytes_to_read == 0) {
                //TBD: process_data
                dev->data.out_buf = handleClientRequest(dev->data.in_buf);
                // reset to default
                free(dev->data.in_buf);
                in_buf_len = 0;
                client_req_bytes_read = 0;
                blob_len_read = false;
            }
        } else {
            in_buf_len = data->data[0];
            // Extra char for null termination
            dev->data.in_buf = (char *)malloc(in_buf_len + 1); 
            dev->data.in_buf[in_buf_len] = '\0';
            client_req_bytes_read = 0;
            blob_len_read = true;
        }
        return;

    default:
        USB_REQ_DEBUG("setting unknown control = %d, length = %d\n",
                      dev->control, data->length);
        return;
    }

    uvc_fill_streaming_control(dev, target, ctrl->bFormatIndex,
                               ctrl->bFrameIndex, ctrl->dwFrameInterval);

    if (dev->control == UVC_VS_COMMIT_CONTROL) {
        const struct uvc_function_config_format *format;
        const struct uvc_function_config_frame *frame;
        struct v4l2_pix_format pixfmt;
        unsigned int fps;

        format = &dev->fc->streaming.formats[target->bFormatIndex - 1];
        frame = &format->frames[target->bFrameIndex - 1];

        dev->fcc = format->fcc;
        dev->width = frame->width;
        dev->height = frame->height;

        memset(&pixfmt, 0, sizeof pixfmt);
        pixfmt.width = frame->width;
        pixfmt.height = frame->height;
        pixfmt.pixelformat = format->fcc;
        pixfmt.field = V4L2_FIELD_NONE;
        if (format->fcc == V4L2_PIX_FMT_MJPEG)
            pixfmt.sizeimage = dev->maxsize * 1.5;

        uvc_stream_set_format(dev->stream, &pixfmt);

        /* fps is guaranteed to be non-zero and thus valid. */
        fps = 1.0 / (target->dwFrameInterval / 10000000.0);
        uvc_stream_set_frame_rate(dev->stream, fps);
    }
}

static void uvc_events_process(void *d) {
    struct uvc_device *dev = d;
    struct v4l2_event v4l2_event;
    const struct uvc_event *uvc_event = (void *)&v4l2_event.u.data;
    struct uvc_request_data resp;
    int ret;

    ret = ioctl(dev->vdev->fd, VIDIOC_DQEVENT, &v4l2_event);
    if (ret < 0) {
        printf("VIDIOC_DQEVENT failed: %s (%d)\n", strerror(errno), errno);
        return;
    }

    memset(&resp, 0, sizeof resp);
    resp.length = -EL2HLT;

    switch (v4l2_event.type) {
    case UVC_EVENT_CONNECT:
    case UVC_EVENT_DISCONNECT:
        return;

    case UVC_EVENT_SETUP:
        uvc_events_process_setup(dev, &uvc_event->req, &resp);
        break;

    case UVC_EVENT_DATA:
        uvc_events_process_data(dev, &uvc_event->data);
        return;

    case UVC_EVENT_STREAMON:
        uvc_stream_enable(dev->stream, 1);
        return;

    case UVC_EVENT_STREAMOFF:
        uvc_stream_enable(dev->stream, 0);
        return;
    }

    ret = ioctl(dev->vdev->fd, UVCIOC_SEND_RESPONSE, &resp);
    if (ret < 0) {
        USB_REQ_DEBUG("UVCIOC_SEND_RESPONSE failed: %s (%d)\n", strerror(errno),
                      errno);
        return;
    }
}

/* ---------------------------------------------------------------------------
 * Initialization and setup
 */

void uvc_events_init(struct uvc_device *dev, struct events *events) {
    struct v4l2_event_subscription sub;

    /* Default to the minimum values. */
    uvc_fill_streaming_control(dev, &dev->probe, 1, 1, 0);
    uvc_fill_streaming_control(dev, &dev->commit, 1, 1, 0);

    memset(&sub, 0, sizeof sub);
    sub.type = UVC_EVENT_SETUP;
    ioctl(dev->vdev->fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
    sub.type = UVC_EVENT_DATA;
    ioctl(dev->vdev->fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
    sub.type = UVC_EVENT_STREAMON;
    ioctl(dev->vdev->fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
    sub.type = UVC_EVENT_STREAMOFF;
    ioctl(dev->vdev->fd, VIDIOC_SUBSCRIBE_EVENT, &sub);

    events_watch_fd(events, dev->vdev->fd, EVENT_EXCEPTION, uvc_events_process,
                    dev);
}

void uvc_set_config(struct uvc_device *dev, struct uvc_function_config *fc) {
    /* FIXME: The maximum size should be specified per format and frame. */
    dev->maxsize = 0;
    dev->fc = fc;
}

int uvc_set_format(struct uvc_device *dev, struct v4l2_pix_format *format) {
    return v4l2_set_format(dev->vdev, format);
}

struct v4l2_device *uvc_v4l2_device(struct uvc_device *dev) {
    /*
	 * TODO: The V4L2 device shouldn't be exposed. We should replace this
	 * with an abstract video sink class when one will be avaiilable.
	 */
    return dev->vdev;
}
