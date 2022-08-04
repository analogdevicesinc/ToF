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
#include "network.h"

#include <functional>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <iostream>

#define RX_BUFFER_BYTES (20996420)
#define MAX_RETRY_CNT 3

enum protocols { PROTOCOL_0 = 0, PROTOCOL_COUNT };

using namespace google::protobuf::io;
using namespace payload;
using namespace std;

struct clientData {
    bool hasFragments;
    std::vector<char> data;
};

static struct lws_protocols protocols[] = {
    {
        "network-protocol",
        Network::callback_function,
        sizeof(clientData),
        RX_BUFFER_BYTES,
    },
    {NULL, NULL, 0, 0} /* terminator */
};

int nBytes = 0;          /*no of bytes sent*/
int recv_data_error = 0; /*flag for recv data*/
char server_msg[] = "Connection Allowed";

/*Declare static members*/
std::vector<lws *> Network::web_socket;
std::vector<lws_context *> Network::context;
ClientRequest Network::send_buff[MAX_CAMERA_NUM];
ServerResponse Network::recv_buff[MAX_CAMERA_NUM];
recursive_mutex Network::m_mutex[MAX_CAMERA_NUM];
mutex Network::mutex_recv[MAX_CAMERA_NUM];
condition_variable_any Network::Cond_Var[MAX_CAMERA_NUM];
condition_variable Network::thread_Cond_Var[MAX_CAMERA_NUM];

bool Network::Send_Successful[MAX_CAMERA_NUM];
bool Network::Data_Received[MAX_CAMERA_NUM];
bool Network::Server_Connected[MAX_CAMERA_NUM];
bool Network::Thread_Detached[MAX_CAMERA_NUM];

/*
* isServer_Connected(): checks if server is connected
* Parameters:        none
* returns:           true  - if server is connected
                     false - if no server is connected
* Desription:   This function checks if server is connected and returns
Server_Connected flag value.
*/
bool Network::isServer_Connected() {
    return Network::Server_Connected[m_connectionId];
}

/*
* isThread_Running(): check if thread created is running
* Parameters:        none
* returns:           true  - if thread is alive
                     false - if thread is not running
* Desription:   This function returns Thread_Running flag value to check if
thread is running.
*/
bool Network::isThread_Running() {
    /*Return true if thread has completed*/
    if (Network::Thread_Running[m_connectionId] == 2) {
        return true;
    } else {
        return false;
    }
}

/*
* isData_Received(): check if data is sent to server and returns Send_Successful
flag value
* Parameters:        none
* returns:           true  - if data has been sent succesfully
                     false - if error in data sending
* Desription:   This function returns Send_Successful flag value
*/
bool Network::isSend_Successful() {
    return Network::Send_Successful[m_connectionId];
}

/*
* isData_Received(): check if data received from server and returns
Data_Received flag value
* Parameters:        none
* returns:           true  - if data received successfully
                     false - if error in data receiving
* Desription:   This function is used to check if any data received from server
*               returns Data_Received flag value.
*/
bool Network::isData_Received() {
    return Network::Data_Received[m_connectionId];
}

/*
* ServerConnect():  intializes the websocket and connects to server
* Parameters:       ip - the ip address of the server to connect to
* returns:          0 - on success
                   -1 - on error
* Desription:   This function initializes the websocket and connects to server.
*/
int Network::ServerConnect(const std::string &ip) {
    struct lws_context_creation_info info;
    memset(&info, 0, sizeof(info));

    info.port = CONTEXT_PORT_NO_LISTEN;
    info.protocols = protocols;
    info.gid = -1;
    info.uid = -1;
    info.pt_serv_buf_size = 4096;

    /*Create a websocket for client*/
    context.at(m_connectionId) = lws_create_context(&info);

    struct lws_client_connect_info ccinfo = {0};
    ccinfo.context = context.at(m_connectionId);
    ccinfo.address = ip.c_str();
    ccinfo.port = 5000;
    ccinfo.path = "/";
    ccinfo.host = lws_canonical_hostname(context.at(m_connectionId));
    ccinfo.origin = "origin";
    ccinfo.protocol = protocols[PROTOCOL_0].name;

    if (!m_connectionId) {
        lws *webSocket = lws_client_connect_via_info(&ccinfo);

        web_socket.clear();
        web_socket.emplace_back(webSocket);
    } else {

        for (size_t i = 0; i < web_socket.size(); i++) {
            if (web_socket.at(i) == nullptr) {
                web_socket.erase(web_socket.begin() + i);
                i--;
            }
        }
        web_socket.push_back(lws_client_connect_via_info(&ccinfo));
    }

    /*Start a new thread to service any pending event on web socket*/

    threadObj[m_connectionId] = std::thread(&Network::call_lws_service, this);

    Network::Thread_Detached[m_connectionId] = true;
    threadObj[m_connectionId].detach();

    /*Wait for thread to be ready and server is connected*/

    std::unique_lock<std::recursive_mutex> mlock(m_mutex[m_connectionId]);

    /*Wait till server is connected or timeout of 3 sec*/
    if (Cond_Var[m_connectionId].wait_for(
            mlock, std::chrono::seconds(3),
            std::bind(&Network::isServer_Connected, this)) == false) {
        Server_Connected[m_connectionId] = false;
        return -1;
    } else if (web_socket.at(m_connectionId) != NULL) {
        /*Wait for Server message to check another client is connected already
         * or not*/
        if (recv_server_data() == 0) {
            /*Data received correctly*/
            if (strcmp(recv_buff[m_connectionId].message().c_str(),
                       server_msg) == 0) {
                /*Server is connected successfully*/
                cout << "Conn established" << endl;
                return 0;
            } else {
                /*Another client is connected already*/
                cout << "Server Message :: "
                     << recv_buff[m_connectionId].message() << endl;
                Server_Connected[m_connectionId] = false;
                return -1;
            }
        } else {
            /*No message received from Server*/
            Server_Connected[m_connectionId] = false;
            return -1;
        }
    } else if (web_socket.at(m_connectionId) == NULL) {
        Server_Connected[m_connectionId] = false;
        return -1;
    }

    return -1;
}

/*
* sendCommand(): send data to server
* Parameters:    none
* returns:       0 - on success
                -1 -  on error
* Desription:    This function is used to send the data to connected server.
*/
int Network::SendCommand() {
    int status = -1;
    uint8_t numRetry = 0;
    int siz = send_buff[m_connectionId].ByteSize();

    recv_buff[m_connectionId].Clear();

    while (numRetry++ < MAX_RETRY_CNT &&
           Server_Connected[m_connectionId] != false) {

        lws_callback_on_writable(web_socket.at(m_connectionId));
        /*Acquire the lock*/
        std::unique_lock<std::recursive_mutex> mlock(m_mutex[m_connectionId]);

        if (Cond_Var[m_connectionId].wait_for(
                mlock, std::chrono::seconds(10),
                std::bind(&Network::isSend_Successful, this)) == false) {
            status = -1; /*timeout occurs*/
#ifdef NW_DEBUG
            cout << "Send Timeout error" << endl;
#endif
            break;
        }

        Send_Successful[m_connectionId] = false;

        if (nBytes < 0) {
            /*Send failed Network issue*/
#ifdef NW_DEBUG
            cout << "Write Error" << endl;
#endif
            status = -1;
            break;
        } else if (nBytes < siz) {
            /*Retry sending command*/
#ifdef NW_DEBUG
            cout << "Retry sending " << numRetry << " times" << endl;
#endif
            status = -1;
        } else if (nBytes == siz) {
#ifdef NW_DEBUG
            cout << "Write Successful" << endl;
#endif
            status = 0;
            break;
        }
    }

    if (Server_Connected[m_connectionId] == false) {
        status = -2;
    }

    return status;
}

/*
* recv_server_data():  receive data from server
* Parameters:   None
* returns:      0  - on success
                -1 -  on error
* Desription:   This function is used to receive the data from connected server
*/
int Network::recv_server_data() {
    int status = -1;
    uint8_t numRetry = 0;
    /*Wait until data received from the server after command execution*/

    while (numRetry++ < MAX_RETRY_CNT &&
           Server_Connected[m_connectionId] != false) {

        /*Acquire the lock*/
        std::unique_lock<std::mutex> mlock(mutex_recv[m_connectionId]);
        if (Cond_Var[m_connectionId].wait_for(
                mlock, std::chrono::seconds(10),
                std::bind(&Network::isData_Received, this)) == true) {
            /*reset the flag value to receive again*/
            Data_Received[m_connectionId] = false;

            if (recv_data_error == 1) {
                /*Retry sending command*/
                if (SendCommand() != 0) {
                    status = -1;
                    break;
                }
            } else if (recv_data_error == 0) {
                /*Data received correctly*/
                status = 0;
                break;
            }

        } else {
            /*No data recvd till time out Retry sending command */
            if (SendCommand() != 0) {
                status = -1;
                break;
            }
        }
    }

    if (Server_Connected[m_connectionId] == false) {
        status = -2;
    }

    send_buff[m_connectionId].Clear();

    return status;
}

/*
 * call_lws_service():  calls websockets library lws_service() api
 * Parameters:   None
 * returns:      None
 * Desription:   This function calls websockets library api to service any
 * pending websocket activity
 */
void Network::call_lws_service() {
    while (1) {
        lws_service(context.at(m_connectionId), 0);
#ifdef NW_DEBUG
        cout << ".";
#endif
        /*Complete the thread if destructor is called*/
        std::lock_guard<std::mutex> guard(thread_mutex[m_connectionId]);

        if (Thread_Running[m_connectionId] == 1) {
#ifdef NW_DEBUG
            cout << "Thread exited" << endl;
#endif
            Thread_Running[m_connectionId] = 2;
            thread_Cond_Var[m_connectionId].notify_all();
            break;
        }
    }
}

/*
* callback_function():  Handles the websocket events
* Parameters:           wsi - websocket instance pointer
                        reasons - websocket event occurred for wsi instance
                        user - pointer to per session user data allocated by
libwebsocket in   - pointer to data len  - length of data
* returns:              0
* Desription:           This function handles the websocket events and take
appropriate action
*/
int Network::callback_function(struct lws *wsi,
                               enum lws_callback_reasons reason, void *user,
                               void *in, size_t len) {
    int connectionId = 0;
    auto status = std::find(web_socket.begin(), web_socket.end(), wsi);
    if (status != web_socket.end()) {
        connectionId =
            static_cast<int>(std::distance(web_socket.begin(), status));
    }

    switch (reason) {
    case LWS_CALLBACK_CLIENT_ESTABLISHED: {
        /*Notify host SDK that server is connected */
        std::lock_guard<std::recursive_mutex> guard(m_mutex[connectionId]);
        Server_Connected[connectionId] = true;
        Cond_Var[connectionId].notify_all();
        break;
    }

    case LWS_CALLBACK_CLIENT_RECEIVE: {
        /* Handle incoming messages here. */
#ifdef NW_DEBUG
        cout << endl << "Rcvd Data len : " << len << endl;
#endif
        std::lock_guard<std::mutex> guard(mutex_recv[connectionId]);

        const size_t remaining = lws_remaining_packet_payload(wsi);
        bool isFinal = lws_is_final_fragment(wsi);

        struct clientData *clientData = static_cast<struct clientData *>(user);

        if (!remaining && isFinal) {
            if (clientData->hasFragments) {
                // apend message
                char *inData = static_cast<char *>(in);
                clientData->data.insert(clientData->data.end(), inData,
                                        inData + len);
                in = static_cast<void *>(clientData->data.data());
                len = clientData->data.size();
            }

            // process message
            google::protobuf::io::ArrayInputStream ais(in,
                                                       static_cast<int>(len));
            CodedInputStream coded_input(&ais);
            recv_buff[connectionId].ParseFromCodedStream(&coded_input);

            recv_data_error = 0;
            Data_Received[connectionId] = true;

            /*Notify the host SDK that data is received from server*/
            Cond_Var[connectionId].notify_all();

        } else {
            // append message
            if (clientData->data.size() == 0) {
                clientData->data.reserve(len + remaining);
            }

            std::cout << "apending data" << std::endl;
            char *inData = static_cast<char *>(in);
            clientData->data.insert(clientData->data.end(), inData,
                                    inData + len);
            clientData->hasFragments = true;
        }

        break;
    }

    case LWS_CALLBACK_CLIENT_WRITEABLE: {
#ifdef NW_DEBUG
        cout << endl << "Client is sending " << send_buff.func_name() << endl;
#endif
        std::lock_guard<std::recursive_mutex> guard(m_mutex[connectionId]);
        if (send_buff[connectionId].func_name().empty()) {
            break;
        }

        /* Get size of packet to be sent*/
        int siz = send_buff[connectionId].ByteSize();
        /*Pre padding of bytes as per websockets*/
        unsigned char *pkt =
            new unsigned char[siz + LWS_SEND_BUFFER_PRE_PADDING];
        unsigned char *pkt_pad = pkt + LWS_SEND_BUFFER_PRE_PADDING;

        google::protobuf::io::ArrayOutputStream aos(pkt_pad, siz);
        CodedOutputStream *coded_output = new CodedOutputStream(&aos);
        send_buff[connectionId].SerializeToCodedStream(coded_output);

        nBytes = lws_write(wsi, pkt_pad, siz, LWS_WRITE_TEXT);

        /*Notify the host SDK that data is sent to server*/
        Send_Successful[connectionId] = true;
        Cond_Var[connectionId].notify_all();

        delete coded_output;
        delete[] pkt;
        send_buff[connectionId].Clear();
        break;
    }

    case LWS_CALLBACK_CLIENT_CLOSED: {
        cout << "Connection Closed" << endl;
        /*Set a flag to indicate server connection is closed abruptly*/
        std::lock_guard<std::recursive_mutex> guard(m_mutex[connectionId]);
        Server_Connected[connectionId] = false;
        web_socket.at(connectionId) = NULL;
        break;
    }

    case LWS_CALLBACK_CLIENT_CONNECTION_ERROR: {
        cout << "Connection Error" << endl;
        web_socket.at(connectionId) = NULL;
        break;
    }

    default:
        break;
    }

    return 0;
}

/*
 * Network():    Initializes the network parameters
 * Parameters:   None
 * Desription:   This function initializes the network parameters
 */
Network::Network(int connectionId) {

    /*Initialize the static flags*/
    Network::Send_Successful[connectionId] = false;
    Network::Data_Received[connectionId] = false;
    Network::Thread_Running[connectionId] = 0;
    Network::Server_Connected[connectionId] = false;
    Network::Thread_Detached[connectionId] = false;

    m_connectionId = connectionId;
    context.emplace_back(nullptr);
}

/*
 * ~Network():   Destructor for network class
 * Parameters:   None
 * Desription:   Destructor for network class
 */
Network::~Network() {
    if (context.at(m_connectionId) != NULL && Thread_Detached[m_connectionId]) {
        /*set a flag to complete the thread */
        std::unique_lock<std::mutex> mlock(thread_mutex[m_connectionId]);
        Thread_Running[m_connectionId] = 1;
        /*wait for thread to finish*/
        thread_Cond_Var[m_connectionId].wait(
            mlock, std::bind(&Network::isThread_Running, this));
        Thread_Detached[m_connectionId] = false;

        lws_context_destroy(context.at(m_connectionId));
    }
}
