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
#include "usb_buffer.pb.h"
#include "connections/usb/usb_depth_sensor.h"
#include "connections/usb/usb_utils.h"
#include "usb_windows_utils.h"

#include "device_utils.h"

#include <atlstr.h>
#include <glog/logging.h>
#include <unordered_map>

struct CalibrationData {
    std::string mode;
    float gain;
    float offset;
    uint16_t *cache;
};

struct UsbDepthSensor::ImplData {
    UsbHandle handle;
    bool opened;
    std::unordered_map<std::string, CalibrationData> calibration_cache;
};

static std::wstring s2ws(const std::string &s) {
    int len;
    int slength = (int)s.length() + 1;
    len = MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, nullptr, 0);
    wchar_t *buf = new wchar_t[len];
    MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, buf, len);
    std::wstring r(buf);
    delete[] buf;
    return r;
}

static aditof::Status getDevice(IBaseFilter **pVideoInputFilter,
                                const std::string &devName) {
    using namespace aditof;
    Status status = Status::OK;

    HRESULT hr;
    BOOL done = FALSE;
    ICreateDevEnum *DevEnum = nullptr;

    hr = CoCreateInstance(CLSID_SystemDeviceEnum, nullptr, CLSCTX_INPROC_SERVER,
                          IID_PPV_ARGS(&DevEnum));
    if (FAILED(hr)) {
        LOG(WARNING) << "Create Device Enumeration Failed";
        return Status::GENERIC_ERROR;
    }

    IEnumMoniker *EnumMoniker = nullptr;
    hr = DevEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory,
                                        &EnumMoniker, 0);

    if (hr != S_OK) {
        DevEnum->Release();
        LOG(WARNING) << "Device Enumeration Error";
        return Status::GENERIC_ERROR;
    }

    IMoniker *Moniker = nullptr;
    ULONG cFetched;
    while (!done && EnumMoniker->Next(1, &Moniker, &cFetched) == S_OK) {
        IPropertyBag *PropBag;
        hr = Moniker->BindToStorage(nullptr, nullptr, IID_PPV_ARGS(&PropBag));

        if (SUCCEEDED(hr)) {
            VARIANT varName;
            VariantInit(&varName);
            hr = PropBag->Read(L"FriendlyName", &varName, nullptr);

            if (SUCCEEDED(hr)) {
                std::string str(static_cast<LPCTSTR>(CString(varName.bstrVal)));
                if (str == devName) {
                    // We found it, so send it back to the caller
                    hr =
                        Moniker->BindToObject(nullptr, nullptr, IID_IBaseFilter,
                                              (void **)pVideoInputFilter);
                    if (!SUCCEEDED(hr)) {
                        LOG(WARNING) << "Failed to bind video input filter";
                    }

                    done = TRUE;
                }
            }
            VariantClear(&varName);
            PropBag->Release();
            PropBag = nullptr;
        }

        Moniker->Release();
        Moniker = nullptr;
    }

    EnumMoniker->Release();
    DevEnum->Release();

    return status;
}

static bool checkSingleByteFormat(GUID FormatType) {
    bool IsSingleByteFormat = true;

    if (FormatType == MEDIASUBTYPE_Y800 || FormatType == MEDIASUBTYPE_Y8 ||
        FormatType == MEDIASUBTYPE_GREY || FormatType == MEDIASUBTYPE_BY8) {
        IsSingleByteFormat = true;
    } else {
        IsSingleByteFormat = false;
    }

    return IsSingleByteFormat;
}

static void NukeDownstream(IBaseFilter *pBF, IGraphBuilder *pGraph) {
    IPin *pP, *pTo;
    ULONG u;
    IEnumPins *pins = nullptr;
    PIN_INFO pininfo;
    HRESULT hr = pBF->EnumPins(&pins);
    pins->Reset();
    while (hr == NOERROR) {
        hr = pins->Next(1, &pP, &u);
        if (hr == S_OK && pP) {
            pP->ConnectedTo(&pTo);
            if (pTo) {
                hr = pTo->QueryPinInfo(&pininfo);
                if (hr == NOERROR) {
                    if (pininfo.dir == PINDIR_INPUT) {
                        NukeDownstream(pininfo.pFilter, pGraph);
                        pGraph->Disconnect(pTo);
                        pGraph->Disconnect(pP);
                        pGraph->RemoveFilter(pininfo.pFilter);
                    }
                    pininfo.pFilter->Release();
                    pininfo.pFilter = nullptr;
                }
                pTo->Release();
            }
            pP->Release();
        }
    }
    if (pins)
        pins->Release();
}

static void destroyGraph(IGraphBuilder *pGraph) {
    HRESULT hr = 0;
    int i = 0;

    while (hr == NOERROR) {
        IEnumFilters *pEnum = nullptr;
        ULONG cFetched;

        // We must get the enumerator again every time because removing a filter
        // from the graph invalidates the enumerator. We always get only the
        // first filter from each enumerator.
        hr = pGraph->EnumFilters(&pEnum);

        IBaseFilter *pFilter = nullptr;

        if (pEnum->Next(1, &pFilter, &cFetched) == S_OK) {
            FILTER_INFO FilterInfo;
            memset(&FilterInfo, 0, sizeof(FilterInfo));
            hr = pFilter->QueryFilterInfo(&FilterInfo);
            FilterInfo.pGraph->Release();

            int count = 0;
            char buffer[255];
            memset(buffer, 0, 255 * sizeof(char));

            while (FilterInfo.achName[count] != 0x00) {
                buffer[count] = (char)FilterInfo.achName[count];
                count++;
            }

            hr = pGraph->RemoveFilter(pFilter);
            if (FAILED(hr)) {
                LOG(WARNING) << "SETUP: pGraph->RemoveFilter() failed.";
            }

            pFilter->Release();
            pFilter = nullptr;
        } else
            break;
        pEnum->Release();
        pEnum = nullptr;
        i++;
    }

    return;
}

UsbDepthSensor::UsbDepthSensor(const std::string &name, const std::string &driverPath)
    : m_driverPath(driverPath), m_implData(new UsbDepthSensor::ImplData) {
    m_implData->handle.pMediaEvent = nullptr;
    m_implData->opened = false;

    m_sensorDetails.connectionType = aditof::ConnectionType::USB;
    m_sensorName = name;
}

UsbDepthSensor::~UsbDepthSensor() {
    HRESULT HR = NOERROR;

    // Check to see if the graph is running, if so stop it.
    if (m_implData->handle.pControl) {
        HR = m_implData->handle.pControl->Pause();

        HR = m_implData->handle.pControl->Stop();
    }

    for (auto it = m_implData->calibration_cache.begin();
         it != m_implData->calibration_cache.begin(); ++it) {
        delete[] it->second.cache;
        it->second.cache = nullptr;
    }

    // Disconnect filters from capture device
    if (m_implData->handle.pVideoInputFilter) {
        NukeDownstream(m_implData->handle.pVideoInputFilter,
                       m_implData->handle.pGraph);
    }

    // Release and zero pointers to our filters etc
    if (m_implData->handle.pDestFilter) {
        m_implData->handle.pDestFilter->Release();
    }

    if (m_implData->handle.pVideoInputFilter) {
        m_implData->handle.pVideoInputFilter->Release();
    }

    if (m_implData->handle.pGrabberF) {
        m_implData->handle.pGrabberF->Release();
    }

    if (m_implData->handle.pGrabber) {
        m_implData->handle.pGrabber->Release();
    }

    if (m_implData->handle.pControl) {
        m_implData->handle.pControl->Release();
    }

    if (m_implData->handle.pMediaEvent) {
        m_implData->handle.pMediaEvent->Release();
    }

    if (m_implData->handle.streamConf) {
        m_implData->handle.streamConf->Release();
    }

    if (m_implData->handle.pAmMediaType) {
        if (m_implData->handle.pAmMediaType->cbFormat != 0) {
            CoTaskMemFree((PVOID)m_implData->handle.pAmMediaType->pbFormat);
            m_implData->handle.pAmMediaType->cbFormat = 0;
            m_implData->handle.pAmMediaType->pbFormat = nullptr;
        }
        if (m_implData->handle.pAmMediaType->pUnk != nullptr) {
            // Unecessary because pUnk should not be used, but safest.
            m_implData->handle.pAmMediaType->pUnk->Release();
            m_implData->handle.pAmMediaType->pUnk = nullptr;
        }
        CoTaskMemFree(m_implData->handle.pAmMediaType);
    }

    // Destroy the graph
    if (m_implData->handle.pGraph) {
        destroyGraph(m_implData->handle.pGraph);
    }

    // Release and zero our capture graph and our main graph
    if (m_implData->handle.pCaptureGraph) {
        m_implData->handle.pCaptureGraph->Release();
    }
    if (m_implData->handle.pGraph) {
        m_implData->handle.pGraph->Release();
    }
}

aditof::Status UsbDepthSensor::open() {
    using namespace aditof;
    Status status = Status::OK;

    LOG(INFO) << "Opening device";

    HRESULT hr;
    GUID CAPTURE_MODE = PIN_CATEGORY_CAPTURE;

    hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);

    hr = CoCreateInstance(CLSID_CaptureGraphBuilder2, nullptr,
                          CLSCTX_INPROC_SERVER, IID_ICaptureGraphBuilder2,
                          (void **)&(m_implData->handle.pCaptureGraph));
    if (FAILED(hr)) {
        LOG(WARNING) << "Failed CoCreateInstance(CLSID_CaptureGraphBuilder2)";
        return Status::GENERIC_ERROR;
    }

    hr = CoCreateInstance(CLSID_FilterGraph, nullptr, CLSCTX_INPROC_SERVER,
                          IID_IGraphBuilder,
                          (void **)&(m_implData->handle.pGraph));
    if (FAILED(hr)) {
        LOG(WARNING) << "Failed CoCreateInstance(CLSID_FilterGraph)";
        return Status::GENERIC_ERROR;
    }

    hr = m_implData->handle.pCaptureGraph->SetFiltergraph(
        m_implData->handle.pGraph);
    if (FAILED(hr)) {
        LOG(WARNING) << "Failed SetFiltergraph";
        return Status::GENERIC_ERROR;
    }

    hr = m_implData->handle.pGraph->QueryInterface(
        IID_IMediaControl, (void **)&(m_implData->handle.pControl));
    if (FAILED(hr)) {
        LOG(WARNING) << "Failed QueryInterface(IID_IMediaControl)";
        return Status::GENERIC_ERROR;
    }

    status = getDevice(&m_implData->handle.pVideoInputFilter, m_driverPath);
    if (status != Status::OK) {
        return status;
    }

    // Query the target about the frame types that are supported by the depth sensor

    // Send request
    usb_payload::ClientRequest requestMsg;
    requestMsg.set_func_name(usb_payload::FunctionName::GET_AVAILABLE_FRAME_TYPES);
    std::string requestStr;
    requestMsg.SerializeToString(&requestStr);
    status = UsbWindowsUtils::uvcExUnitSendRequest(m_implData->handle.pVideoInputFilter, requestStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Request to get available frame types failed";
        return status;
    }

    // Read UVC gadget response
    std::string responseStr;
    status = UsbWindowsUtils::uvcExUnitGetResponse(m_implData->handle.pVideoInputFilter, responseStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Response for get available frame types request failed";
        return status;
    }
    usb_payload::ServerResponse responseMsg;
    bool parsed = responseMsg.ParseFromString(responseStr);
    if (!parsed) {
        LOG(ERROR) << "Failed to deserialize string containing UVC gadget response";
        return aditof::Status::INVALID_ARGUMENT;
    }

    if (responseMsg.status() != usb_payload::Status::OK) {
        LOG(ERROR) << "Get available frame types operation failed on UVC gadget";
        return static_cast<aditof::Status>(responseMsg.status());
    }

    // If request and response went well, extract data from response
    UsbUtils::protoMsgToDepthSensorFrameTypes(m_depthSensorFrameTypes,
        responseMsg.available_frame_types());

    std::wstring stemp = s2ws(m_driverPath);
    hr = m_implData->handle.pGraph->AddFilter(
        m_implData->handle.pVideoInputFilter, stemp.c_str());
    if (FAILED(hr)) {
        LOG(WARNING) << "ADI TOF Camera cannot be opened";
        return Status::GENERIC_ERROR;
    }

    IAMStreamConfig *streamConfTest = nullptr;
    hr = m_implData->handle.pCaptureGraph->FindInterface(
        &PIN_CATEGORY_PREVIEW, &MEDIATYPE_Video,
        m_implData->handle.pVideoInputFilter, IID_IAMStreamConfig,
        (void **)&streamConfTest);
    if (FAILED(hr)) {
        // TO DO: old IO library allowed this to fail. Investigate why. Until then don't bother showing this error.
        // LOG(WARNING) << "Failed FindInterface(PIN_CATEGORY_PREVIEW)";
    } else {
        CAPTURE_MODE = PIN_CATEGORY_PREVIEW;
        streamConfTest->Release();
        streamConfTest = nullptr;
    }

    hr = m_implData->handle.pCaptureGraph->FindInterface(
        &CAPTURE_MODE, &MEDIATYPE_Video, m_implData->handle.pVideoInputFilter,
        IID_IAMStreamConfig, (void **)&(m_implData->handle.streamConf));
    if (FAILED(hr)) {
        LOG(WARNING) << "Failed FindInterface(CAPTURE_MODE)";
        return Status::GENERIC_ERROR;
    }

    hr = m_implData->handle.streamConf->GetFormat(
        &(m_implData->handle.pAmMediaType));
    if (FAILED(hr)) {
        LOG(WARNING) << "Failed to get format from streamConf";
        return Status::GENERIC_ERROR;
    }

    hr = CoCreateInstance(CLSID_SampleGrabber, nullptr, CLSCTX_INPROC_SERVER,
                          IID_IBaseFilter,
                          (void **)&(m_implData->handle.pGrabberF));
    if (FAILED(hr)) {
        LOG(WARNING) << "Could not Create Sample Grabber - CoCreateInstance()";
        return Status::GENERIC_ERROR;
    }

    hr = m_implData->handle.pGraph->AddFilter(m_implData->handle.pGrabberF,
                                              L"Sample Grabber");
    if (FAILED(hr)) {
        LOG(WARNING) << "Could not add Sample Grabber - AddFilter()";
        return Status::GENERIC_ERROR;
    }

    hr = m_implData->handle.pGrabberF->QueryInterface(
        IID_ISampleGrabber, (void **)&(m_implData->handle.pGrabber));
    if (FAILED(hr)) {
        LOG(WARNING) << "ERROR: Could not query SampleGrabber";
        return Status::GENERIC_ERROR;
    }

    hr = m_implData->handle.pGrabber->SetOneShot(FALSE);
    hr = m_implData->handle.pGrabber->SetBufferSamples(TRUE);
    if (FAILED(hr)) {
        LOG(WARNING) << "Fail SetBuffer";
        return Status::GENERIC_ERROR;
    }

    AM_MEDIA_TYPE mt;
    ZeroMemory(&mt, sizeof(AM_MEDIA_TYPE));

    mt.majortype = MEDIATYPE_Video;
    // Included conditional based format for Y16
    if (checkSingleByteFormat(m_implData->handle.pAmMediaType->subtype) ||
        (m_implData->handle.pAmMediaType->subtype == MEDIASUBTYPE_Y16)) {
        mt.subtype = m_implData->handle.pAmMediaType->subtype;
    } else
        mt.subtype = MEDIASUBTYPE_RGB24; // Making it RGB24, does conversion
                                         // from YUV to RGB Included conditional
                                         // based format for Y16 - end

    mt.formattype = FORMAT_VideoInfo;

    hr = m_implData->handle.pGrabber->SetMediaType(&mt);

    // NULL RENDERER//
    // used to give the video stream somewhere to go to.
    hr = CoCreateInstance(CLSID_NullRenderer, nullptr, CLSCTX_INPROC_SERVER,
                          IID_IBaseFilter,
                          (void **)(&(m_implData->handle.pDestFilter)));
    if (FAILED(hr)) {
        LOG(WARNING) << "ERROR: Could not create filter - NullRenderer";
        return Status::GENERIC_ERROR;
    }

    hr = m_implData->handle.pGraph->AddFilter(m_implData->handle.pDestFilter,
                                              L"NullRenderer");
    if (FAILED(hr)) {
        LOG(WARNING) << "ERROR: Could not add filter - NullRenderer";
        return Status::GENERIC_ERROR;
    }

    // RENDER STREAM//
    // This is where the stream gets put together.
    hr = m_implData->handle.pCaptureGraph->RenderStream(
        &PIN_CATEGORY_PREVIEW, &MEDIATYPE_Video,
        m_implData->handle.pVideoInputFilter, m_implData->handle.pGrabberF,
        m_implData->handle.pDestFilter);

    if (FAILED(hr)) {
        LOG(WARNING) << "ERROR: Could not connect pins - RenderStream()";
        return Status::GENERIC_ERROR;
    }

    // Try setting the sync source to null - and make it run as fast as possible
    IMediaFilter *pMediaFilter = nullptr;
    hr = m_implData->handle.pGraph->QueryInterface(IID_IMediaFilter,
                                                   (void **)&pMediaFilter);
    if (FAILED(hr)) {
        LOG(WARNING) << "ERROR: Could not get IID_IMediaFilter interface";
        return Status::GENERIC_ERROR;
    } else {
        pMediaFilter->SetSyncSource(nullptr);
        pMediaFilter->Release();
    }

    m_implData->handle.pCB = new SampleGrabberCallback();
    hr = m_implData->handle.pGrabber->SetCallback(m_implData->handle.pCB, 1);

    m_implData->opened = true;

    return status;
}

aditof::Status UsbDepthSensor::start() {
    using namespace aditof;
    Status status = Status::OK;

    if (nullptr == m_implData->handle.pControl) {
        LOG(WARNING) << "USB interface not active";
        return Status::UNAVAILABLE;
    }

    // RUN THE STREAM
    HRESULT hr = m_implData->handle.pControl->Run();
    if (FAILED(hr)) {
        LOG(WARNING) << "ERROR: Could not start graph";
        return Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status UsbDepthSensor::stop() {
    using namespace aditof;
    Status status = Status::OK;

    LOG(INFO) << "Stopping device";

    if (nullptr == m_implData->handle.pControl) {
        LOG(WARNING) << "USB interface not active";
        return Status::UNAVAILABLE;
    }

    HRESULT hr = m_implData->handle.pControl->Stop();
    if (FAILED(hr)) {
        LOG(WARNING) << "ERROR: Could not stop graph";
        return Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status UsbDepthSensor::getAvailableFrameTypes(
    std::vector<aditof::DepthSensorFrameType> &types) {
    using namespace aditof;
    Status status = Status::OK;

    types = m_depthSensorFrameTypes;

    return status;
}

aditof::Status
UsbDepthSensor::setFrameType(const aditof::DepthSensorFrameType &type) {
    using namespace aditof;
    Status status = Status::OK;
    HRESULT hr = m_implData->handle.streamConf->GetFormat(
        &(m_implData->handle.pAmMediaType));
    if (FAILED(hr)) {
        LOG(WARNING) << "failed 7";
        return Status::GENERIC_ERROR;
    }
    // Send the frame type and all its content all the way to target
    usb_payload::ClientRequest requestMsg;
    auto frameTypeMsg = requestMsg.mutable_frame_type();
    UsbUtils::depthSensorFrameTypeToProtoMsg(type, frameTypeMsg);
    // Send request
    requestMsg.set_func_name(usb_payload::FunctionName::SET_FRAME_TYPE);
    std::string requestStr;
    requestMsg.SerializeToString(&requestStr);
    status = UsbWindowsUtils::uvcExUnitSendRequest(
        m_implData->handle.pVideoInputFilter, requestStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Set frame type operation failed on UVC gadget";
        return status;
    }
    VIDEOINFOHEADER *pVih = reinterpret_cast<VIDEOINFOHEADER *>(
        m_implData->handle.pAmMediaType->pbFormat);
    HEADER(pVih)->biWidth = type.width;
    HEADER(pVih)->biHeight = type.height;
    hr = m_implData->handle.streamConf->SetFormat(
        m_implData->handle.pAmMediaType);
    if (FAILED(hr)) {
        LOG(WARNING) << "Could not set requested resolution (Frame Index)\n";
        return Status::GENERIC_ERROR;
    }
    return status;
}

aditof::Status UsbDepthSensor::program(const uint8_t *firmware, size_t size) {
    using namespace aditof;

    ExUnitHandle handle;

    HRESULT hr = UsbWindowsUtils::UvcFindNodeAndGetControl(
        &handle, &m_implData->handle.pVideoInputFilter);
    if (hr != S_OK) {
        LOG(WARNING) << "Failed to find node and get control. Error: "
                     << std::hex << hr;
        return Status::GENERIC_ERROR;
    }

    OAFilterState state;
    m_implData->handle.pControl->GetState(1, &state);
    if (state == _FilterState::State_Running) {
        hr = m_implData->handle.pControl->Pause();
    }

    size_t written_bytes = 0;
    BYTE buf[MAX_BUF_SIZE];

    while (written_bytes < size) {
        if ((size - written_bytes) > MAX_PACKET_SIZE) {
            memcpy(&buf[2], &firmware[written_bytes], MAX_PACKET_SIZE);
            buf[0] = 0x01;
            buf[1] = MAX_PACKET_SIZE;
            written_bytes += MAX_PACKET_SIZE;
        } else {
            memset(buf, 0, MAX_BUF_SIZE);
            buf[0] = 0x02;
            buf[1] = static_cast<BYTE>(size - written_bytes);
            memcpy(&buf[2], &firmware[written_bytes], size - written_bytes);
            written_bytes = size;
        }
        hr = UsbWindowsUtils::UvcExUnitSetProperty(&handle, 1, &buf[0],
                                                   MAX_BUF_SIZE);
        if (FAILED(hr)) {
            LOG(WARNING) << " Error in Programming AFE";
            return Status::GENERIC_ERROR;
        }
    }

    return Status::OK;
}

aditof::Status UsbDepthSensor::getFrame(uint16_t *buffer) {
    using namespace aditof;
    Status status = Status::OK;

    int retryCount = 0;
    HRESULT hr;

    VIDEOINFOHEADER *pVi = reinterpret_cast<VIDEOINFOHEADER *>(
        m_implData->handle.pAmMediaType->pbFormat);
    int currentWidth = HEADER(pVi)->biWidth;
    int currentHeight = HEADER(pVi)->biHeight;

    while (retryCount < 1000) {
        if (m_implData->handle.pCB->newFrame == 1) {
            long bufferSize = currentWidth * currentHeight * 2;
            hr = m_implData->handle.pGrabber->GetCurrentBuffer(
                (long *)&bufferSize, (long *)buffer);
            if (hr != S_OK) {
                LOG(WARNING) << "Incorrect Buffer Size allocated, Allocate "
                                "bigger buffer";
                continue;
            } else {
                EnterCriticalSection(&m_implData->handle.pCB->critSection);
                m_implData->handle.pCB->newFrame = false;
                LeaveCriticalSection(&m_implData->handle.pCB->critSection);
                break;
            }
        } else {
            Sleep(1);
            retryCount++;
        }
    }

    return retryCount >= 1000 ? Status::GENERIC_ERROR : status;
}

aditof::Status UsbDepthSensor::readRegisters(const uint16_t *address,
                                                uint16_t *data, size_t length, bool burst /*=true*/) {
    using namespace aditof;

    Status status = Status::OK;
    
    // TO DO: is it required to call UvcFindNodeAndGetControl here?

    // Construct request message
    usb_payload::ClientRequest requestMsg;
    requestMsg.set_func_name(usb_payload::FunctionName::READ_REGISTERS);
    requestMsg.add_func_int32_param(static_cast<::google::int32>(length));
    requestMsg.add_func_int32_param(static_cast<::google::int32>(burst));
    requestMsg.add_func_bytes_param(address, burst ? 2 : length * sizeof(uint16_t));

    // Send request
    std::string requestStr;
    requestMsg.SerializeToString(&requestStr);
    status = UsbWindowsUtils::uvcExUnitSendRequest(m_implData->handle.pVideoInputFilter, requestStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Request to read registers failed";
        return status;
    }

    // Read UVC gadget response
    std::string responseStr;
    status = UsbWindowsUtils::uvcExUnitGetResponse(m_implData->handle.pVideoInputFilter, responseStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to get response of the request to read registers";
        return status;
    }
    usb_payload::ServerResponse responseMsg;
    bool parsed = responseMsg.ParseFromString(responseStr);
    if (!parsed) {
        LOG(ERROR) << "Failed to deserialize string containing UVC gadget response";
        return aditof::Status::INVALID_ARGUMENT;
    }

    if (responseMsg.status() != usb_payload::Status::OK) {
        LOG(ERROR) << "Read registers operation failed on UVC gadget";
        return static_cast<aditof::Status>(responseMsg.status());
    }

    // If request and response went well, extract data from response
    memcpy(data, responseMsg.bytes_payload(0).c_str(), responseMsg.bytes_payload(0).length());

    return Status::OK;
}

aditof::Status UsbDepthSensor::writeRegisters(const uint16_t *address,
                                                 const uint16_t *data,
                                                 size_t length, bool burst /*=true*/) {
    using namespace aditof;

    Status status = Status::OK;

    // TO DO: is it required to call UvcFindNodeAndGetControl here?

    // Construct request message
    usb_payload::ClientRequest requestMsg;
    requestMsg.set_func_name(usb_payload::FunctionName::WRITE_REGISTERS);
    requestMsg.add_func_int32_param(static_cast<::google::int32>(length));
    requestMsg.add_func_int32_param(static_cast<::google::int32>(burst));
    requestMsg.add_func_bytes_param(address, burst ? 2 : length * sizeof(uint16_t));
    requestMsg.add_func_bytes_param(data, length * sizeof(uint16_t));

    // Send request
    std::string requestStr;
    requestMsg.SerializeToString(&requestStr);
    status = UsbWindowsUtils::uvcExUnitSendRequest(m_implData->handle.pVideoInputFilter, requestStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Request to write registers failed";
        return status;
    }

    // Read UVC gadget response
    std::string responseStr;
    status = UsbWindowsUtils::uvcExUnitGetResponse(m_implData->handle.pVideoInputFilter, responseStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to get response of the request to write registers";
        return status;
    }
    usb_payload::ServerResponse responseMsg;
    bool parsed = responseMsg.ParseFromString(responseStr);
    if (!parsed) {
        LOG(ERROR) << "Failed to deserialize string containing UVC gadget response";
        return aditof::Status::INVALID_ARGUMENT;
    }

    if (responseMsg.status() != usb_payload::Status::OK) {
        LOG(ERROR) << "Read registers operation failed on UVC gadget";
        return static_cast<aditof::Status>(responseMsg.status());
    }

    return Status::OK;
}

aditof::Status UsbDepthSensor::getAvailableControls(std::vector<std::string> &controls) const {
    using namespace aditof;

    Status status = Status::OK;

    // Construct request message
    usb_payload::ClientRequest requestMsg;
    requestMsg.set_func_name(usb_payload::FunctionName::GET_AVAILABLE_CONTROLS);

    // Send request
    std::string requestStr;
    requestMsg.SerializeToString(&requestStr);
    status = UsbWindowsUtils::uvcExUnitSendRequest(m_implData->handle.pVideoInputFilter, requestStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Request to get available controls failed";
        return status;
    }

    // Read UVC gadget response
    std::string responseStr;
    status = UsbWindowsUtils::uvcExUnitGetResponse(m_implData->handle.pVideoInputFilter, responseStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to get response of the request to get controls";
        return status;
    }
    usb_payload::ServerResponse responseMsg;
    bool parsed = responseMsg.ParseFromString(responseStr);
    if (!parsed) {
        LOG(ERROR) << "Failed to deserialize string containing UVC gadget response";
        return aditof::Status::INVALID_ARGUMENT;
    }

    if (responseMsg.status() != usb_payload::Status::OK) {
        LOG(ERROR) << "Get available controls operation failed on UVC gadget";
        return static_cast<aditof::Status>(responseMsg.status());
    }

    if (status == Status::OK) {
        controls.clear();

        for (int i = 0; i < responseMsg.strings_payload_size(); i++) {
            std::string controlName = responseMsg.strings_payload(i);
            controls.push_back(controlName);
        }
    }

    return Status::OK;
}

aditof::Status UsbDepthSensor::setControl(const std::string &control,
                               const std::string &value)
{
    using namespace aditof;

    Status status = Status::OK;

    // Construct request message
    usb_payload::ClientRequest requestMsg;
    requestMsg.set_func_name(usb_payload::FunctionName::SET_CONTROL);
    requestMsg.add_func_strings_param(control);
    requestMsg.add_func_strings_param(value);

    // Send request
    std::string requestStr;
    requestMsg.SerializeToString(&requestStr);
    status = UsbWindowsUtils::uvcExUnitSendRequest(m_implData->handle.pVideoInputFilter, requestStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Request to set control failed: " << control;
        return status;
    }

    // Read UVC gadget response
    std::string responseStr;
    status = UsbWindowsUtils::uvcExUnitGetResponse(m_implData->handle.pVideoInputFilter, responseStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to get response of the request to set control: " << control;
        return status;
    }
    usb_payload::ServerResponse responseMsg;
    bool parsed = responseMsg.ParseFromString(responseStr);
    if (!parsed) {
        LOG(ERROR) << "Failed to deserialize string containing UVC gadget response";
        return aditof::Status::INVALID_ARGUMENT;
    }

    if (responseMsg.status() != usb_payload::Status::OK) {
        LOG(ERROR) << "Set control:" << control << " operation failed on UVC gadget";
        return static_cast<aditof::Status>(responseMsg.status());
    }

    return Status::OK;
}

aditof::Status UsbDepthSensor::getControl(const std::string &control,
                               std::string &value) const
{
    using namespace aditof;

    Status status = Status::OK;

    // Construct request message
    usb_payload::ClientRequest requestMsg;
    requestMsg.set_func_name(usb_payload::FunctionName::GET_CONTROL);
    requestMsg.add_func_strings_param(control);

    // Send request
    std::string requestStr;
    requestMsg.SerializeToString(&requestStr);
    status = UsbWindowsUtils::uvcExUnitSendRequest(m_implData->handle.pVideoInputFilter, requestStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Request to get control: " << control << " failed";
        return status;
    }

    // Read UVC gadget response
    std::string responseStr;
    status = UsbWindowsUtils::uvcExUnitGetResponse(m_implData->handle.pVideoInputFilter, responseStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to get response of the request to get control: " << control;
        return status;
    }
    usb_payload::ServerResponse responseMsg;
    bool parsed = responseMsg.ParseFromString(responseStr);
    if (!parsed) {
        LOG(ERROR) << "Failed to deserialize string containing UVC gadget response";
        return aditof::Status::INVALID_ARGUMENT;
    }

    if (responseMsg.status() != usb_payload::Status::OK) {
        LOG(ERROR) << "Get control: " << control << " operation failed on UVC gadget";
        return static_cast<aditof::Status>(responseMsg.status());
    }

    value = responseMsg.strings_payload(0);

    return Status::OK;
}

aditof::Status
UsbDepthSensor::getDetails(aditof::SensorDetails &details) const {
    details = m_sensorDetails;
    return aditof::Status::OK;
}

aditof::Status UsbDepthSensor::getHandle(void **handle) {
    if (m_implData->opened) {
        *handle = &m_implData->handle;
        return aditof::Status::OK;
    } else {
        *handle = nullptr;
        LOG(ERROR) << "Won't return the handle. Device hasn't been opened yet.";
        return aditof::Status::UNAVAILABLE;
    }
}

aditof::Status UsbDepthSensor::getName(std::string &name) const {
    name = m_sensorName;
    return aditof::Status::OK;
}

aditof::Status UsbDepthSensor::adsd3500_read_cmd(uint16_t cmd, uint16_t *data){
    using namespace aditof;

    Status status = Status::OK;

    // Construct request message
    usb_payload::ClientRequest requestMsg;
    requestMsg.set_func_name(usb_payload::FunctionName::ADSD3500_READ_CMD);
    requestMsg.add_func_int32_param(static_cast<::google::int32>(cmd));

    // Send request
    std::string requestStr;
    requestMsg.SerializeToString(&requestStr);
    status = UsbWindowsUtils::uvcExUnitSendRequest(m_implData->handle.pVideoInputFilter, requestStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Request to read registers failed";
        return status;
    }

    // Read UVC gadget response
    std::string responseStr;
    status = UsbWindowsUtils::uvcExUnitGetResponse(m_implData->handle.pVideoInputFilter, responseStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to get response of the request to read registers";
        return status;
    }
    usb_payload::ServerResponse responseMsg;
    bool parsed = responseMsg.ParseFromString(responseStr);
    if (!parsed) {
        LOG(ERROR) << "Failed to deserialize string containing UVC gadget response";
        return aditof::Status::INVALID_ARGUMENT;
    }

    if (responseMsg.status() != usb_payload::Status::OK) {
        LOG(ERROR) << "Read registers operation failed on UVC gadget";
        return static_cast<aditof::Status>(responseMsg.status());
    }

    // If request and response went well, extract data from response
    *data = responseMsg.int32_payload(0);

    return Status::OK;
}

aditof::Status UsbDepthSensor::adsd3500_write_cmd(uint16_t cmd, uint16_t data) {
    using namespace aditof;

    Status status = Status::OK;

    // Construct request message
    usb_payload::ClientRequest requestMsg;
    requestMsg.set_func_name(usb_payload::FunctionName::ADSD3500_WRITE_CMD);
    requestMsg.add_func_int32_param(static_cast<::google::int32>(cmd));
    requestMsg.add_func_int32_param(static_cast<::google::int32>(data));

    // Send request
    std::string requestStr;
    requestMsg.SerializeToString(&requestStr);
    status = UsbWindowsUtils::uvcExUnitSendRequest(m_implData->handle.pVideoInputFilter, requestStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Request to write registers failed";
        return status;
    }

    // Read UVC gadget response
    std::string responseStr;
    status = UsbWindowsUtils::uvcExUnitGetResponse(m_implData->handle.pVideoInputFilter, responseStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to get response of the request to write registers";
        return status;
    }
    usb_payload::ServerResponse responseMsg;
    bool parsed = responseMsg.ParseFromString(responseStr);
    if (!parsed) {
        LOG(ERROR) << "Failed to deserialize string containing UVC gadget response";
        return aditof::Status::INVALID_ARGUMENT;
    }

    if (responseMsg.status() != usb_payload::Status::OK) {
        LOG(ERROR) << "Read registers operation failed on UVC gadget";
        return static_cast<aditof::Status>(responseMsg.status());
    }

    return Status::OK;
}

aditof::Status UsbDepthSensor::adsd3500_read_payload_cmd(uint32_t cmd, uint8_t* readback_data, uint16_t payload_len) {
        using namespace aditof;

    Status status = Status::OK;

    // Construct request message
    usb_payload::ClientRequest requestMsg;
    requestMsg.set_func_name(usb_payload::FunctionName::ADSD3500_READ_PAYLOAD_CMD);
    requestMsg.add_func_int32_param(static_cast<::google::int32>(cmd));
    requestMsg.add_func_int32_param(static_cast<::google::int32>(payload_len));

    // Send request
    std::string requestStr;
    requestMsg.SerializeToString(&requestStr);
    status = UsbWindowsUtils::uvcExUnitSendRequest(m_implData->handle.pVideoInputFilter, requestStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Request to read registers failed";
        return status;
    }

    // Read UVC gadget response
    std::string responseStr;
    status = UsbWindowsUtils::uvcExUnitGetResponse(m_implData->handle.pVideoInputFilter, responseStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to get response of the request to read registers";
        return status;
    }
    usb_payload::ServerResponse responseMsg;
    bool parsed = responseMsg.ParseFromString(responseStr);
    if (!parsed) {
        LOG(ERROR) << "Failed to deserialize string containing UVC gadget response";
        return aditof::Status::INVALID_ARGUMENT;
    }

    if (responseMsg.status() != usb_payload::Status::OK) {
        LOG(ERROR) << "Read registers operation failed on UVC gadget";
        return static_cast<aditof::Status>(responseMsg.status());
    }

    // If request and response went well, extract data from response
    memcpy(readback_data, responseMsg.bytes_payload(0).c_str(), responseMsg.bytes_payload(0).length());

    return Status::OK;
}

aditof::Status UsbDepthSensor::adsd3500_read_payload(uint8_t* payload, uint16_t payload_len) {
    using namespace aditof;

    return Status::UNAVAILABLE;
}

aditof::Status UsbDepthSensor::adsd3500_write_payload_cmd(uint32_t cmd, uint8_t* payload, uint16_t payload_len) {
    using namespace aditof;

    Status status = Status::OK;

    // Construct request message
    usb_payload::ClientRequest requestMsg;
    requestMsg.set_func_name(usb_payload::FunctionName::ADSD3500_WRITE_PAYLOAD_CMD);
    requestMsg.add_func_int32_param(static_cast<::google::int32>(cmd));
    requestMsg.add_func_int32_param(static_cast<::google::int32>(payload_len));
    requestMsg.add_func_bytes_param(payload, payload_len);

    // Send request
    std::string requestStr;
    requestMsg.SerializeToString(&requestStr);
    status = UsbWindowsUtils::uvcExUnitSendRequest(m_implData->handle.pVideoInputFilter, requestStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Request to write registers failed";
        return status;
    }

    // Read UVC gadget response
    std::string responseStr;
    status = UsbWindowsUtils::uvcExUnitGetResponse(m_implData->handle.pVideoInputFilter, responseStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to get response of the request to write registers";
        return status;
    }
    usb_payload::ServerResponse responseMsg;
    bool parsed = responseMsg.ParseFromString(responseStr);
    if (!parsed) {
        LOG(ERROR) << "Failed to deserialize string containing UVC gadget response";
        return aditof::Status::INVALID_ARGUMENT;
    }

    if (responseMsg.status() != usb_payload::Status::OK) {
        LOG(ERROR) << "Read registers operation failed on UVC gadget";
        return static_cast<aditof::Status>(responseMsg.status());
    }

    return Status::OK;
}

aditof::Status UsbDepthSensor::adsd3500_write_payload(uint8_t* payload, uint16_t payload_len) {
    using namespace aditof;

    Status status = Status::OK;

    // Construct request message
    usb_payload::ClientRequest requestMsg;
    requestMsg.set_func_name(usb_payload::FunctionName::ADSD3500_WRITE_PAYLOAD_CMD);
    requestMsg.add_func_int32_param(static_cast<::google::int32>(payload_len));
    requestMsg.add_func_bytes_param(payload, payload_len);

    // Send request
    std::string requestStr;
    requestMsg.SerializeToString(&requestStr);
    status = UsbWindowsUtils::uvcExUnitSendRequest(m_implData->handle.pVideoInputFilter, requestStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Request to write registers failed";
        return status;
    }

    // Read UVC gadget response
    std::string responseStr;
    status = UsbWindowsUtils::uvcExUnitGetResponse(m_implData->handle.pVideoInputFilter, responseStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to get response of the request to write registers";
        return status;
    }
    usb_payload::ServerResponse responseMsg;
    bool parsed = responseMsg.ParseFromString(responseStr);
    if (!parsed) {
        LOG(ERROR) << "Failed to deserialize string containing UVC gadget response";
        return aditof::Status::INVALID_ARGUMENT;
    }

    if (responseMsg.status() != usb_payload::Status::OK) {
        LOG(ERROR) << "Read registers operation failed on UVC gadget";
        return static_cast<aditof::Status>(responseMsg.status());
    }

    return Status::OK;
}
