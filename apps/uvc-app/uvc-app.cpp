
#include <glog/logging.h>

#include "tof-sdk-interface.h"
#include <aditof/version.h>

extern "C" {
#include "configfs.h"
#include "events.h"
#include "stream.h"
#include "v4l2-source.h"
}

int main(int argc, char *argv[]) {
    char *function = NULL;
    char *cap_device_1 = NULL;
    const char *cap_device = "/dev/video0";
    struct uvc_function_config *fc;
    struct uvc_stream *stream = NULL;
    struct video_source *src = NULL;
    struct events events;
    int ret = 0;

    // Init google logging system
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;

    DLOG(INFO) << argv[0] << " "
               << "has started";
    DLOG(INFO) << "This UVC instance is using aditof sdk version: "
               << ADITOF_API_VERSION;

    ret = init_tof_sdk(cap_device_1);
    if (ret)
        return ret;

    fc = configfs_parse_uvc_function(function);
    if (!fc) {
        DLOG(INFO) << "Failed to identify function configuration\n";
        return 1;
    }

    /*
	 * Create the events handler.
	 */
    events_init(&events);

    /* Create and initialize a video source. */
    src = v4l2_video_source_create(cap_device);
    if (src == NULL) {
        ret = 1;
        goto done;
    }

    v4l2_video_source_init(src, &events);

    /* Create and initialise the stream. */
    stream = uvc_stream_new(fc->video);
    if (stream == NULL) {
        ret = 1;
        goto done;
    }

    uvc_stream_set_event_handler(stream, &events);
    uvc_stream_set_video_source(stream, src);
    uvc_stream_init_uvc(stream, fc);

    /* Main capture loop */
    events_loop(&events);

done:
    /* Cleanup */
    uvc_stream_delete(stream);
    video_source_destroy(src);
    events_cleanup(&events);
    configfs_free_uvc_function(fc);

    return ret;
}
