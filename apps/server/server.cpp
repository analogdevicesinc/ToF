/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "server.h"
#include "aditof/aditof.h"
#include "aditof/sensor_enumerator_factory.h"
#include "aditof/sensor_enumerator_interface.h"
#include "buffer.pb.h"

#include "../../sdk/src/connections/target/v4l_buffer_access_interface.h"

#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif
#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <future>
#include <iostream>
#include <linux/videodev2.h>
#include <map>
#include <queue>
#include <string>
#include <sys/time.h>
#include <thread>

using namespace google::protobuf::io;

static const int FRAME_PREPADDING_BYTES = 2;
static int interrupted = 0;

/* Available sensors */
std::vector<std::shared_ptr<aditof::DepthSensorInterface>> depthSensors;
bool sensors_are_created = false;
bool clientEngagedWithSensors = false;

std::unique_ptr<aditof::SensorEnumeratorInterface> sensorsEnumerator;

/* Server only works with one depth sensor */
std::shared_ptr<aditof::DepthSensorInterface> camDepthSensor;
std::shared_ptr<aditof::V4lBufferAccessInterface> sensorV4lBufAccess;
int processedFrameSize;

static payload::ClientRequest buff_recv;
static payload::ServerResponse buff_send;

//sending frames separately without serializing it
uint8_t *buff_frame_to_be_captured = nullptr;
uint8_t *buff_frame_to_send = nullptr;
unsigned int buff_frame_length;
bool m_frame_ready = false;

static std::map<std::string, api_Values> s_map_api_Values;
static void Initialize();
static void data_transaction();
void invoke_sdk_api(payload::ClientRequest buff_recv);
static bool Client_Connected = false;
static bool no_of_client_connected = false;
bool latest_sent_msg_is_was_buffered = false;
static std::queue<aditof::Adsd3500Status> adsd3500InterruptsQueue;

// A test mode that server can be set to. After sending one frame from sensor to host, it will repeat
// sending the same frame over and over without acquiring any other frame from sensor. This allows
// testing the network link speed because it eliminates operations on target such as getting the frame
// from v4l2 interface, passing the frame through depth compute and any deep copying.
static bool sameFrameEndlessRepeat = false;

// Variables for synchronizing the main thread and the thread responsible for capturing frames from hardware
std::mutex
    frameMutex; // used for making sure operations on queue are not done simultaneously by the 2 threads
std::condition_variable
    cvGetFrame; // used for threads to signal when to start capturing a frame or when a frame has become available
bool goCaptureFrame =
    false; // Flag used by main thread to tell the frame capturing thread to start capturing a frame
bool frameCaptured =
    false; // Flag used by frame capturing thread to tell the main thread that a frame has become available
std::thread
    frameCaptureThread; // The thread instance for the capturing frame thread
bool keepCaptureThreadAlive =
    false; // Flag used by frame capturing thread to know whether to continue or finish

std::unique_ptr<zmq::socket_t> server_socket;
uint32_t max_send_frames = 10;
std::atomic<bool> running(false);
std::atomic<bool> stop_flag(false);
std::thread data_transaction_thread;
std::mutex connection_mtx;
std::mutex mtx;
std::condition_variable cv;
std::thread stream_thread;
static std::unique_ptr<zmq::context_t> context;
static std::unique_ptr<zmq::socket_t> server_cmd;
static std::unique_ptr<zmq::socket_t> monitor_socket;
bool send_async = false;

struct clientData {
    bool hasFragments;
    std::vector<char> data;
};

void close_zmq_connection() {
    if (server_socket) {
        server_socket->close(); // Close the socket
        server_socket.reset();  // Release the unique pointer
    }

    LOG(INFO) << "ZMQ Client Connection closed.";
}

void stream_zmq_frame() {

    LOG(INFO) << "stream_frame thread running in the background.";
    zmq::pollitem_t items[] = {
        {static_cast<void *>(*server_socket), 0, ZMQ_POLLOUT, 0}};

    running = true;

    while (true) {

        zmq::poll(items, 1, FRAME_TIMEOUT); // Poll with a timeout of 200 ms

        if (stop_flag.load()) {
            LOG(INFO) << "stream_frame thread is exiting.";
            break;
        }
        // 1. Wait for frame to be captured on the other thread
        std::unique_lock<std::mutex> lock(frameMutex);
        cvGetFrame.wait(lock, []() { return frameCaptured; });

        // 2. Get your hands on the captured frame
        std::swap(buff_frame_to_send, buff_frame_to_be_captured);
        frameCaptured = false;
        lock.unlock();

        // 3. Trigger the other thread to capture another frame while we do stuff with current frame
        {
            std::lock_guard<std::mutex> lock(frameMutex);
            goCaptureFrame = true;
        }
        cvGetFrame.notify_one();

        if (!server_socket) {
            LOG(ERROR) << "ZMQ server socket is not initialized!";
            break;
        }
        zmq::message_t message(buff_frame_length);
        memcpy(message.data(), buff_frame_to_send, buff_frame_length);
        if (items[0].revents & ZMQ_POLLOUT) {
            server_socket->send(message, zmq::send_flags::none);
            // LOG(INFO) << "Frame sent successfully size : " << buff_frame_length;
        } else {
            LOG(INFO) << "Socket not ready, Dropping Frames";
        }
    }

    {
        std::lock_guard<std::mutex> thread_lock(mtx);
        running = false;
    }

    cv.notify_all();

    LOG(INFO) << "stream_zmq_frame thread stopped successfully.";
}

void start_stream_thread() {

    // Reset the stop flag
    stop_flag.store(false);

    keepCaptureThreadAlive = true;

    if (stream_thread.joinable()) {
        stream_thread.join(); // Ensure the previous thread is cleaned up
    }

    stream_thread = std::thread(stream_zmq_frame); // Assign thread
}

void stop_stream_thread() {

    if (!running) {
        return; // If the thread is already stopped, exit the function.
    }

    {
        std::lock_guard<std::mutex> lock(mtx);
        stop_flag.store(true);
    }

    {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [] { return running.load() == false; });
    }

    // Flush the messages
    if (server_socket) {
        server_socket->setsockopt(ZMQ_LINGER, 0);
    }

    if (stream_thread.joinable()) {
        stream_thread.join(); // Ensure the thread exits cleanly
    }
}

aditof::SensorInterruptCallback callback = [](aditof::Adsd3500Status status) {
    adsd3500InterruptsQueue.push(status);
    DLOG(INFO) << "ADSD3500 interrupt occured: status = " << status;
};

// Function executed in the capturing frame thread
static void captureFrameFromHardware() {
    while (keepCaptureThreadAlive) {

        // 1. Wait for the signal to start capturing a new frame
        std::unique_lock<std::mutex> lock(frameMutex);
        cvGetFrame.wait(
            lock, [] { return goCaptureFrame || !keepCaptureThreadAlive; });

        if (!keepCaptureThreadAlive) {
            break;
        }

        // 2. The signal has been received, now go capture the frame
        goCaptureFrame = false;

        // Send frames to PC via ZMQ socket.
        aditof::Status status =
            camDepthSensor->getFrame((uint16_t *)buff_frame_to_be_captured);
        if (status == aditof::Status::UNREACHABLE) {
            LOG(INFO) << "The capture_frame thread is stopped. Stopping the "
                         "stream_frame thread";
            break;
        }

        // 3. Notify others that there is a new frame available
        frameCaptured = true;
        lock.unlock();
        cvGetFrame.notify_one();
    }

    return;
}

static void cleanup_sensors() {
    // Stop the frame capturing thread
    if (frameCaptureThread.joinable()) {
        keepCaptureThreadAlive = false;
        { std::lock_guard<std::mutex> lock(frameMutex); }
        cvGetFrame.notify_one();
        frameCaptureThread.join();
    }

    camDepthSensor->adsd3500_unregister_interrupt_callback(callback);
    sensorV4lBufAccess.reset();
    camDepthSensor.reset();

    while (!adsd3500InterruptsQueue.empty()) {
        adsd3500InterruptsQueue.pop();
    }
    sensors_are_created = false;
    clientEngagedWithSensors = false;
}

void server_event(std::unique_ptr<zmq::socket_t> &monitor) {
    while (!interrupted) {
        zmq::pollitem_t items[] = {
            {static_cast<void *>(monitor->handle()), 0, ZMQ_POLLIN, 0}};

        int rc;
        do {
            rc = zmq_poll(items, 1, 1000);
        } while (rc == -1 && zmq_errno() == EINTR);

        if (items[0].revents & ZMQ_POLLIN) {
            zmq_event_t event;
            zmq::message_t msg;
            monitor->recv(msg);
            memcpy(&event, msg.data(), sizeof(event));
            Network::callback_function(event);
        }
    }
}

int Network::callback_function(const zmq_event_t &event) {
    switch (event.event) {
    case ZMQ_EVENT_CONNECTED: {

        break;
    }
    case ZMQ_EVENT_CLOSED:
        std::cout << "Closed connection " << std::endl;
        if (Client_Connected && !no_of_client_connected) {
            std::cout << "Connection Closed" << std::endl;
            if (clientEngagedWithSensors) {
                cleanup_sensors();
                clientEngagedWithSensors = false;
            }
            stop_stream_thread();
            Client_Connected = false;
        } else {
            std::cout << "Another Client Connection Closed" << std::endl;
            no_of_client_connected = false;
        }
        break;
    case ZMQ_EVENT_CONNECT_RETRIED:
        std::cout << "Connection retried to " << std::endl;
        break;
    case ZMQ_EVENT_ACCEPTED:
        buff_send.Clear();
        if (!Client_Connected) {
            std::cout << "Conn Established" << std::endl;
            {
                std::unique_lock<std::mutex> lock(connection_mtx);
                Client_Connected = true;
            }
            buff_send.set_message("Connection Allowed");
        } else {
            std::cout << "Another client connected" << std::endl;
            no_of_client_connected = true;
        }
        break;
    case ZMQ_EVENT_DISCONNECTED: {
        if (Client_Connected && !no_of_client_connected) {
            std::cout << "Connection Closed" << std::endl;
            if (clientEngagedWithSensors) {
                cleanup_sensors();
                clientEngagedWithSensors = false;
            }
            stop_stream_thread();
            Client_Connected = false;
        } else {
            std::cout << "Another Client Connection Closed" << std::endl;
            no_of_client_connected = false;
        }
        break;
    }
    default:
#ifdef NW_DEBUG
        std::cout << "Event: " << event.event << " on " << addr << std::endl;
#endif
        break;
    }
    return 0;
}

void data_transaction() {

    while (!interrupted) {

        zmq::message_t request;

        std::unique_lock<std::mutex> lock(connection_mtx);
        if (Client_Connected) {
            if (server_cmd->recv(request, zmq::recv_flags::dontwait)) {
                google::protobuf::io::ArrayInputStream ais(request.data(),
                                                           request.size());
                google::protobuf::io::CodedInputStream coded_input(&ais);
                buff_recv.ParseFromCodedStream(&coded_input);
                invoke_sdk_api(buff_recv);

                // Preparing to send the data
                unsigned int siz = buff_send.ByteSize();
                unsigned char *pkt = new unsigned char[siz];

                google::protobuf::io::ArrayOutputStream aos(pkt, siz);
                google::protobuf::io::CodedOutputStream coded_output(&aos);
                buff_send.SerializeToCodedStream(&coded_output);

                // Create a zmq message
                zmq::message_t reply(pkt, siz);
                if (server_cmd->send(reply, zmq::send_flags::none)) {
#ifdef NW_DEBUG
                    LOG(INFO) << "Data is sent ";
#endif
                }
                delete[] pkt;
            }
        }
    }
}

void sigint_handler(int) {
    interrupted = 1;

    if (sensors_are_created) {
        cleanup_sensors();
    }
    clientEngagedWithSensors = false;

    stop_stream_thread();
    close_zmq_connection();

    if (server_cmd) {
        server_cmd->close();
        server_cmd.reset();
    }
    if (monitor_socket) {
        monitor_socket->close();
        monitor_socket.reset();
    }
    if (context) {
        context->close();
        context.reset();
    }
}

int main(int argc, char *argv[]) {
    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);

    LOG(INFO) << "Server built \n"
              << "with SDK version: " << aditof::getApiVersion()
              << " | branch: " << aditof::getBranchVersion()
              << " | commit: " << aditof::getCommitVersion();

    context = std::make_unique<zmq::context_t>(2);
    server_cmd = std::make_unique<zmq::socket_t>(*context, ZMQ_REP);
    // Bind the socket
    server_cmd->bind("tcp://*:5556");
    std::string monitor_endpoint = "inproc://monitor";
    zmq_socket_monitor(server_cmd->handle(), "inproc://monitor", ZMQ_EVENT_ALL);

    monitor_socket = std::make_unique<zmq::socket_t>(*context, ZMQ_PAIR);
    // Connect the monitor socket
    monitor_socket->connect("inproc://monitor");

    // run thread to receive data
    data_transaction_thread = std::thread(data_transaction);
    data_transaction_thread.detach();

    Initialize();

    if (sensors_are_created) {
        cleanup_sensors();
    }

    server_event(monitor_socket);

    return 0;
}

void invoke_sdk_api(payload::ClientRequest buff_recv) {
    buff_send.Clear();
    buff_send.set_server_status(::payload::ServerStatus::REQUEST_ACCEPTED);

    DLOG(INFO) << buff_recv.func_name() << " function";

    switch (s_map_api_Values[buff_recv.func_name()]) {

    case FIND_SENSORS: {
        if (!sensors_are_created) {
            sensorsEnumerator =
                aditof::SensorEnumeratorFactory::buildTargetSensorEnumerator();
            if (!sensorsEnumerator) {
                std::string errMsg =
                    "Failed to create a target sensor enumerator";
                LOG(WARNING) << errMsg;
                buff_send.set_message(errMsg);
                buff_send.set_status(static_cast<::payload::Status>(
                    aditof::Status::UNAVAILABLE));
                break;
            }

            sensorsEnumerator->searchSensors();
            sensorsEnumerator->getDepthSensors(depthSensors);
            sensors_are_created = true;
        }

        /* Add information about available sensors */

        // Depth sensor
        if (depthSensors.size() < 1) {
            buff_send.set_message("No depth sensors are available");
            buff_send.set_status(::payload::Status::UNREACHABLE);
            break;
        }

        camDepthSensor = depthSensors.front();
        auto pbSensorsInfo = buff_send.mutable_sensors_info();
        sensorV4lBufAccess =
            std::dynamic_pointer_cast<aditof::V4lBufferAccessInterface>(
                camDepthSensor);

        std::string name;
        camDepthSensor->getName(name);
        auto pbDepthSensorInfo = pbSensorsInfo->mutable_image_sensors();
        pbDepthSensorInfo->set_name(name);

        std::string kernelversion;
        std::string ubootversion;
        std::string sdversion;
        auto cardVersion = buff_send.mutable_card_image_version();

        sensorsEnumerator->getKernelVersion(kernelversion);
        cardVersion->set_kernelversion(kernelversion);
        sensorsEnumerator->getUbootVersion(ubootversion);
        cardVersion->set_ubootversion(ubootversion);
        sensorsEnumerator->getSdVersion(sdversion);
        cardVersion->set_sdversion(sdversion);

        // This server is now subscribing for interrupts of ADSD3500
        aditof::Status registerCbStatus =
            camDepthSensor->adsd3500_register_interrupt_callback(callback);
        if (registerCbStatus != aditof::Status::OK) {
            LOG(WARNING) << "Could not register callback";
            // TBD: not sure whether to send this error to client or not
        }

        buff_send.set_status(
            static_cast<::payload::Status>(aditof::Status::OK));
        break;
    }

    case OPEN: {
        aditof::Status status = camDepthSensor->open();
        buff_send.set_status(static_cast<::payload::Status>(status));
        clientEngagedWithSensors = true;

        // At this stage, start the capturing frames thread
        keepCaptureThreadAlive = true;
        frameCaptureThread = std::thread(captureFrameFromHardware);

        break;
    }

    case START: {
        aditof::Status status = camDepthSensor->start();

        // When in test mode, capture 2 frames. 1st might be corrupt after a ADSD3500 reset.
        // 2nd frame will be the one sent over and over again by server in test mode.
        if (sameFrameEndlessRepeat) {
            for (int i = 0; i < 2; ++i) {
                status =
                    camDepthSensor->getFrame((uint16_t *)(buff_frame_to_send));
                if (status != aditof::Status::OK) {
                    LOG(ERROR) << "Failed to get frame!";
                }
            }
        } else { // When in normal mode, trigger the capture thread to fetch a frame
            {
                std::lock_guard<std::mutex> lock(frameMutex);
                goCaptureFrame = true;
            }
            cvGetFrame.notify_one();
        }
        static zmq::context_t zmq_context(1);
        server_socket = std::make_unique<zmq::socket_t>(zmq_context,
                                                        zmq::socket_type::push);
        server_socket->setsockopt(ZMQ_SNDHWM, (int *)&max_send_frames,
                                  sizeof(max_send_frames));
        server_socket->bind("tcp://*:5555");
        LOG(INFO) << "ZMQ server socket connection established.";
        if (send_async == true) {
            start_stream_thread(); // Start the stream_frame thread .
        }

        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case STOP: {
        aditof::Status status = camDepthSensor->stop();
        buff_send.set_status(static_cast<::payload::Status>(status));

        if (send_async == true) {
            stop_stream_thread();
        }

        close_zmq_connection();

        break;
    }

    case GET_AVAILABLE_MODES: {
        std::vector<uint8_t> aditofModes;
        aditof::Status status = camDepthSensor->getAvailableModes(aditofModes);
        for (auto &modeName : aditofModes) {
            buff_send.add_int32_payload(modeName);
        }
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case GET_MODE_DETAILS: {
        aditof::DepthSensorModeDetails frameDetails;
        uint8_t modeName = buff_recv.func_int32_param(0);
        aditof::Status status =
            camDepthSensor->getModeDetails(modeName, frameDetails);
        auto protoContent = buff_send.mutable_depth_sensor_mode_details();
        protoContent->set_mode_number(frameDetails.modeNumber);
        protoContent->set_pixel_format_index(frameDetails.pixelFormatIndex);
        protoContent->set_frame_width_in_bytes(frameDetails.frameWidthInBytes);
        protoContent->set_frame_height_in_bytes(
            frameDetails.frameHeightInBytes);
        protoContent->set_base_resolution_width(
            frameDetails.baseResolutionWidth);
        protoContent->set_base_resolution_height(
            frameDetails.baseResolutionHeight);
        protoContent->set_metadata_size(frameDetails.metadataSize);
        protoContent->set_is_pcm(frameDetails.isPCM);
        protoContent->set_number_of_phases(frameDetails.numberOfPhases);
        for (int i = 0; i < frameDetails.frameContent.size(); i++) {
            protoContent->add_frame_content(frameDetails.frameContent.at(i));
        }
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case SET_MODE_BY_INDEX: {

        uint8_t mode = buff_recv.func_int32_param(0);

        aditof::Status status = camDepthSensor->setMode(mode);
        if (status == aditof::Status::OK) {
            aditof::DepthSensorModeDetails aditofModeDetail;
            status = camDepthSensor->getModeDetails(mode, aditofModeDetail);
            if (status != aditof::Status::OK) {
                buff_send.set_status(static_cast<::payload::Status>(status));
                break;
            }

            int width_tmp = aditofModeDetail.baseResolutionWidth;
            int height_tmp = aditofModeDetail.baseResolutionHeight;

            if (aditofModeDetail.isPCM) {
                processedFrameSize =
                    width_tmp * height_tmp * aditofModeDetail.numberOfPhases;

            } else {
#ifdef DUAL
                if (mode == 1 || mode == 0) {
                    processedFrameSize = width_tmp * height_tmp * 2;
                } else {
                    processedFrameSize = width_tmp * height_tmp * 4;
                }
#else
                processedFrameSize = width_tmp * height_tmp * 4;
#endif
            }

            if (buff_frame_to_send != nullptr) {
                delete[] buff_frame_to_send;
                buff_frame_to_send = nullptr;
            }

            buff_frame_to_send =
                new uint8_t[processedFrameSize * sizeof(uint16_t)];

            if (buff_frame_to_be_captured != nullptr) {
                delete[] buff_frame_to_be_captured;
                buff_frame_to_be_captured = nullptr;
            }

            buff_frame_to_be_captured =
                new uint8_t[processedFrameSize * sizeof(uint16_t)];

            buff_frame_length = processedFrameSize * 2;
        }

        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case SET_MODE: {
        aditof::DepthSensorModeDetails aditofModeDetail;
        aditofModeDetail.modeNumber = buff_recv.mode_details().mode_number();
        aditofModeDetail.pixelFormatIndex =
            buff_recv.mode_details().pixel_format_index();
        aditofModeDetail.frameWidthInBytes =
            buff_recv.mode_details().frame_width_in_bytes();
        aditofModeDetail.frameHeightInBytes =
            buff_recv.mode_details().frame_height_in_bytes();
        aditofModeDetail.baseResolutionWidth =
            buff_recv.mode_details().base_resolution_width();
        aditofModeDetail.baseResolutionHeight =
            buff_recv.mode_details().base_resolution_height();
        aditofModeDetail.metadataSize =
            buff_recv.mode_details().metadata_size();

        for (int i = 0; i < buff_recv.mode_details().frame_content_size();
             i++) {
            aditofModeDetail.frameContent.emplace_back(
                buff_recv.mode_details().frame_content(i));
        }

        aditof::Status status = camDepthSensor->setMode(aditofModeDetail);

        if (status == aditof::Status::OK) {
            int width_tmp = aditofModeDetail.baseResolutionWidth;
            int height_tmp = aditofModeDetail.baseResolutionHeight;

            if (aditofModeDetail.isPCM) {
                processedFrameSize =
                    width_tmp * height_tmp * aditofModeDetail.numberOfPhases;
            } else {
                processedFrameSize = width_tmp * height_tmp * 4;
            }

            if (buff_frame_to_send != nullptr) {
                delete[] buff_frame_to_send;
                buff_frame_to_send = nullptr;
            }

            buff_frame_to_send =
                new uint8_t[processedFrameSize * sizeof(uint16_t)];

            if (buff_frame_to_be_captured != nullptr) {
                delete[] buff_frame_to_be_captured;
                buff_frame_to_be_captured = nullptr;
            }

            buff_frame_to_be_captured =
                new uint8_t[processedFrameSize * sizeof(uint16_t)];

            buff_frame_length = processedFrameSize * 2;
        }

        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case GET_FRAME: {
        if (sameFrameEndlessRepeat) {
            m_frame_ready = true;
            zmq::message_t message(buff_frame_length);
            memcpy(message.data(), buff_frame_to_send, buff_frame_length);
            server_socket->send(message, zmq::send_flags::none);
            buff_send.set_status(
                static_cast<::payload::Status>(aditof::Status::OK));
            break;
        } else {
            // 1. Wait for frame to be captured on the other thread
            std::unique_lock<std::mutex> lock(frameMutex);
            cvGetFrame.wait(lock, []() { return frameCaptured; });

            // 2. Get your hands on the captured frame
            std::swap(buff_frame_to_send, buff_frame_to_be_captured);
            frameCaptured = false;
            lock.unlock();

            // 3. Trigger the other thread to capture another frame while we do stuff with current frame
            {
                std::lock_guard<std::mutex> lock(frameMutex);
                goCaptureFrame = true;
            }
            cvGetFrame.notify_one();

            // 4. Send current frame over network

            zmq::message_t message(buff_frame_length);
            memcpy(message.data(), buff_frame_to_send, buff_frame_length);
            server_socket->send(message, zmq::send_flags::none);

            m_frame_ready = true;

            buff_send.set_status(payload::Status::OK);
            break;
        }
    }

    case GET_AVAILABLE_CONTROLS: {
        std::vector<std::string> aditofControls;

        aditof::Status status =
            camDepthSensor->getAvailableControls(aditofControls);
        for (const auto &aditofControl : aditofControls) {
            buff_send.add_strings_payload(aditofControl);
        }
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case SET_CONTROL: {
        std::string controlName = buff_recv.func_strings_param(0);
        std::string controlValue = buff_recv.func_strings_param(1);
        aditof::Status status =
            camDepthSensor->setControl(controlName, controlValue);
        if (controlName == "netlinktest") {
            sameFrameEndlessRepeat = controlValue == "1";
        }
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case GET_CONTROL: {
        std::string controlName = buff_recv.func_strings_param(0);
        std::string controlValue;
        aditof::Status status =
            camDepthSensor->getControl(controlName, controlValue);
        buff_send.add_strings_payload(controlValue);
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case SET_SENSOR_CONFIGURATION: {
        std::string sensorConf = buff_recv.func_strings_param(0);
        aditof::Status status =
            camDepthSensor->setSensorConfiguration(sensorConf);
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case INIT_TARGET_DEPTH_COMPUTE: {
        aditof::Status status = camDepthSensor->initTargetDepthCompute(
            (uint8_t *)buff_recv.func_bytes_param(0).c_str(),
            static_cast<uint16_t>(buff_recv.func_int32_param(0)),
            (uint8_t *)buff_recv.func_bytes_param(1).c_str(),
            static_cast<uint16_t>(buff_recv.func_int32_param(1)));

        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case ADSD3500_READ_CMD: {
        uint16_t cmd = static_cast<uint16_t>(buff_recv.func_int32_param(0));
        uint16_t data;
        unsigned int usDelay =
            static_cast<unsigned int>(buff_recv.func_int32_param(1));

        aditof::Status status =
            camDepthSensor->adsd3500_read_cmd(cmd, &data, usDelay);
        if (status == aditof::Status::OK) {
            buff_send.add_int32_payload(static_cast<::google::int32>(data));
        }

        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case ADSD3500_WRITE_CMD: {
        uint16_t cmd = static_cast<uint16_t>(buff_recv.func_int32_param(0));
        uint16_t data = static_cast<uint16_t>(buff_recv.func_int32_param(1));
        uint32_t usDelay = static_cast<uint32_t>(buff_recv.func_int32_param(2));

        aditof::Status status =
            camDepthSensor->adsd3500_write_cmd(cmd, data, usDelay);
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case ADSD3500_READ_PAYLOAD_CMD: {
        uint32_t cmd = static_cast<uint32_t>(buff_recv.func_int32_param(0));
        uint16_t payload_len =
            static_cast<uint16_t>(buff_recv.func_int32_param(1));
        uint8_t *data = new uint8_t[payload_len];

        memcpy(data, buff_recv.func_bytes_param(0).c_str(),
               4 * sizeof(uint8_t));
        aditof::Status status =
            camDepthSensor->adsd3500_read_payload_cmd(cmd, data, payload_len);
        if (status == aditof::Status::OK) {
            buff_send.add_bytes_payload(data, payload_len);
        }

        delete[] data;
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case ADSD3500_READ_PAYLOAD: {
        uint16_t payload_len =
            static_cast<uint16_t>(buff_recv.func_int32_param(0));
        uint8_t *data = new uint8_t[payload_len];

        aditof::Status status =
            camDepthSensor->adsd3500_read_payload(data, payload_len);
        if (status == aditof::Status::OK) {
            buff_send.add_bytes_payload(data, payload_len);
        }

        delete[] data;
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case ADSD3500_WRITE_PAYLOAD_CMD: {
        uint32_t cmd = static_cast<uint32_t>(buff_recv.func_int32_param(0));
        uint16_t payload_len =
            static_cast<uint16_t>(buff_recv.func_int32_param(1));
        uint8_t *data = new uint8_t[payload_len];

        memcpy(data, buff_recv.func_bytes_param(0).c_str(), payload_len);
        aditof::Status status =
            camDepthSensor->adsd3500_write_payload_cmd(cmd, data, payload_len);

        delete[] data;
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case ADSD3500_WRITE_PAYLOAD: {
        uint16_t payload_len =
            static_cast<uint16_t>(buff_recv.func_int32_param(0));
        uint8_t *data = new uint8_t[payload_len];

        memcpy(data, buff_recv.func_bytes_param(0).c_str(), payload_len);
        aditof::Status status =
            camDepthSensor->adsd3500_write_payload(data, payload_len);

        delete[] data;
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case ADSD3500_GET_STATUS: {
        int chipStatus;
        int imagerStatus;

        aditof::Status status =
            camDepthSensor->adsd3500_get_status(chipStatus, imagerStatus);
        if (status == aditof::Status::OK) {
            buff_send.add_int32_payload(chipStatus);
            buff_send.add_int32_payload(imagerStatus);
        }

        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case GET_INTERRUPTS: {
        while (!adsd3500InterruptsQueue.empty()) {
            buff_send.add_int32_payload((int)adsd3500InterruptsQueue.front());
            adsd3500InterruptsQueue.pop();
        }

        buff_send.set_status(
            static_cast<::payload::Status>(aditof::Status::OK));
        break;
    }

    case HANG_UP: {
        if (sensors_are_created) {
            cleanup_sensors();
        }
        clientEngagedWithSensors = false;

        break;
    }

    case GET_DEPTH_COMPUTE_PARAM: {
        std::map<std::string, std::string> ini_params;
        aditof::Status status =
            camDepthSensor->getDepthComputeParams(ini_params);
        if (status == aditof::Status::OK) {
            buff_send.add_strings_payload(ini_params["abThreshMin"]);
            buff_send.add_strings_payload(ini_params["abSumThresh"]);
            buff_send.add_strings_payload(ini_params["confThresh"]);
            buff_send.add_strings_payload(ini_params["radialThreshMin"]);
            buff_send.add_strings_payload(ini_params["radialThreshMax"]);
            buff_send.add_strings_payload(ini_params["jblfApplyFlag"]);
            buff_send.add_strings_payload(ini_params["jblfWindowSize"]);
            buff_send.add_strings_payload(ini_params["jblfGaussianSigma"]);
            buff_send.add_strings_payload(ini_params["jblfExponentialTerm"]);
            buff_send.add_strings_payload(ini_params["jblfMaxEdge"]);
            buff_send.add_strings_payload(ini_params["jblfABThreshold"]);
            buff_send.add_strings_payload(ini_params["headerSize"]);
        }
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case SET_DEPTH_COMPUTE_PARAM: {
        std::map<std::string, std::string> ini_params;
        ini_params["abThreshMin"] = buff_recv.func_strings_param(0);
        ini_params["abSumThresh"] = buff_recv.func_strings_param(1);
        ini_params["confThresh"] = buff_recv.func_strings_param(2);
        ini_params["radialThreshMin"] = buff_recv.func_strings_param(3);
        ini_params["radialThreshMax"] = buff_recv.func_strings_param(4);
        ini_params["jblfApplyFlag"] = buff_recv.func_strings_param(5);
        ini_params["jblfWindowSize"] = buff_recv.func_strings_param(6);
        ini_params["jblfGaussianSigma"] = buff_recv.func_strings_param(7);
        ini_params["jblfExponentialTerm"] = buff_recv.func_strings_param(8);
        ini_params["jblfMaxEdge"] = buff_recv.func_strings_param(9);
        ini_params["jblfABThreshold"] = buff_recv.func_strings_param(10);

        aditof::Status status =
            camDepthSensor->setDepthComputeParams(ini_params);
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case GET_INI_ARRAY: {
        int mode = buff_recv.func_int32_param(0);
        std::string iniStr;

        aditof::Status status =
            camDepthSensor->getIniParamsArrayForMode(mode, iniStr);

        if (status == aditof::Status::OK) {
            buff_send.add_strings_payload(iniStr);
        }
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }
    case SERVER_CONNECT: {
        if (!no_of_client_connected) {
            buff_send.set_message("Connection Allowed");
        } else {
            buff_send.set_message("Only 1 client connection allowed");
        }
        break;
    }

    case RECV_ASYNC: {
        send_async = true;
        buff_send.set_message("send_async");

        break;
    }

    default: {
        std::string msgErr = "Function not found";
        std::cout << msgErr << "\n";

        buff_send.set_message(msgErr);
        buff_send.set_server_status(::payload::ServerStatus::REQUEST_UNKNOWN);
        break;
    }
    } // switch

    buff_send.set_interrupt_occured(!adsd3500InterruptsQueue.empty());

    buff_recv.Clear();
}

void Initialize() {
    s_map_api_Values["FindSensors"] = FIND_SENSORS;
    s_map_api_Values["Open"] = OPEN;
    s_map_api_Values["Start"] = START;
    s_map_api_Values["Stop"] = STOP;
    s_map_api_Values["GetAvailableModes"] = GET_AVAILABLE_MODES;
    s_map_api_Values["GetModeDetails"] = GET_MODE_DETAILS;
    s_map_api_Values["SetModeByIndex"] = SET_MODE_BY_INDEX;
    s_map_api_Values["SetMode"] = SET_MODE;
    s_map_api_Values["GetFrame"] = GET_FRAME;
    s_map_api_Values["GetAvailableControls"] = GET_AVAILABLE_CONTROLS;
    s_map_api_Values["SetControl"] = SET_CONTROL;
    s_map_api_Values["GetControl"] = GET_CONTROL;
    s_map_api_Values["SetSensorConfiguration"] = SET_SENSOR_CONFIGURATION;
    s_map_api_Values["InitTargetDepthCompute"] = INIT_TARGET_DEPTH_COMPUTE;
    s_map_api_Values["Adsd3500ReadCmd"] = ADSD3500_READ_CMD;
    s_map_api_Values["Adsd3500WriteCmd"] = ADSD3500_WRITE_CMD;
    s_map_api_Values["Adsd3500ReadPayloadCmd"] = ADSD3500_READ_PAYLOAD_CMD;
    s_map_api_Values["Adsd3500ReadPayload"] = ADSD3500_READ_PAYLOAD;
    s_map_api_Values["Adsd3500WritePayloadCmd"] = ADSD3500_WRITE_PAYLOAD_CMD;
    s_map_api_Values["Adsd3500WritePayload"] = ADSD3500_WRITE_PAYLOAD;
    s_map_api_Values["Adsd3500GetStatus"] = ADSD3500_GET_STATUS;
    s_map_api_Values["GetInterrupts"] = GET_INTERRUPTS;
    s_map_api_Values["HangUp"] = HANG_UP;
    s_map_api_Values["GetDepthComputeParam"] = GET_DEPTH_COMPUTE_PARAM;
    s_map_api_Values["SetDepthComputeParam"] = SET_DEPTH_COMPUTE_PARAM;
    s_map_api_Values["GetIniArray"] = GET_INI_ARRAY;
    s_map_api_Values["ServerConnect"] = SERVER_CONNECT;
    s_map_api_Values["RecvAsync"] = RECV_ASYNC;
}