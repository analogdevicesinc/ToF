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
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <zmq.hpp>

#define RX_BUFFER_BYTES (20996420)

#define FRAME_TIMEOUT 200

enum api_Values {
    API_NOT_DEFINED,
    FIND_SENSORS,
    OPEN,
    START,
    STOP,
    GET_AVAILABLE_MODES,
    GET_MODE_DETAILS,
    SET_MODE,
    SET_MODE_BY_INDEX,
    GET_FRAME,
    GET_AVAILABLE_CONTROLS,
    SET_CONTROL,
    GET_CONTROL,
    INIT_TARGET_DEPTH_COMPUTE,
    ADSD3500_READ_CMD,
    ADSD3500_WRITE_CMD,
    ADSD3500_READ_PAYLOAD_CMD,
    ADSD3500_READ_PAYLOAD,
    ADSD3500_WRITE_PAYLOAD_CMD,
    ADSD3500_WRITE_PAYLOAD,
    ADSD3500_GET_STATUS,
    GET_INTERRUPTS,
    HANG_UP,
    GET_DEPTH_COMPUTE_PARAM,
    SET_DEPTH_COMPUTE_PARAM,
    SET_SENSOR_CONFIGURATION,
    GET_INI_ARRAY,
    SERVER_CONNECT,
    RECV_ASYNC
};

enum protocols { PROTOCOL_EXAMPLE, PROTOCOL_COUNT };

class Network {
  public:
    Network();
    static int callback_function(const zmq_event_t &event);
};
