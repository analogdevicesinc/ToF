
#include <glog/logging.h>
#include <unistd.h>

#include <aditof/depth_sensor_interface.h>
#include <aditof/sensor_enumerator_factory.h>
#include <aditof/sensor_enumerator_interface.h>
#include <aditof/storage_interface.h>
#include <aditof/temperature_sensor_interface.h>
#include <aditof/version.h>
#include "../../sdk/src/connections/target/adsd3100_sensor.h"

extern "C" {
#include "configfs.h"
#include "events.h"
#include "stream.h"
#include "v4l2-source.h"
}

int main(int argc, char *argv[]) {
    char *function = NULL;
    const char *cap_device = "/dev/video0";
    struct uvc_function_config *fc;
    struct uvc_stream *stream = NULL;
    struct video_source *src = NULL;
    struct events events;
    int ret = 0;

    /* Available sensors */
    std::vector<std::shared_ptr<aditof::DepthSensorInterface>> depthSensors;
    std::vector<std::shared_ptr<aditof::StorageInterface>> storages;
    std::vector<std::shared_ptr<aditof::TemperatureSensorInterface>>
        temperatureSensors;

    /* UVC only works with one depth sensor */
    std::shared_ptr<aditof::DepthSensorInterface> camDepthSensor;

    // Init google logging system
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;

    DLOG(INFO) << argv[0] << " "
               << "has started";

    auto sensorsEnumerator =
        aditof::SensorEnumeratorFactory::buildTargetSensorEnumerator();
    if (!sensorsEnumerator) {
        DLOG(INFO) << "Failed to construct a sensors enumerator!";
        return 1;
    }

    sensorsEnumerator->searchSensors();
    sensorsEnumerator->getDepthSensors(depthSensors);
    sensorsEnumerator->getStorages(storages);
    sensorsEnumerator->getTemperatureSensors(temperatureSensors);

    if (depthSensors.size() < 1) {
        DLOG(INFO) << "No camera sensor are available!";
        return 1;
    }

    camDepthSensor = depthSensors[0];
    auto depthSensor =
        std::dynamic_pointer_cast<Adsd3100Sensor>(camDepthSensor);
    if (!depthSensor) {
        DLOG(INFO) << "Camera sensor is not of type V4lBufferAccessInterface!";
        return 1;
    } else {
		DLOG(INFO)<<"Camera found, with driver path: "<<depthSensor->getDriverPath() ;
    }
    std::string availableSensorsBlob;
    // Build a message about available sensors types to be sent to the UVC client
    aditof::SensorDetails camSensorDetails;
    camDepthSensor->getDetails(camSensorDetails);

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
