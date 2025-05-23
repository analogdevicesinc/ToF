/********************************************************************************/
/*                                                                              */
/* Copyright (c) Microsoft Corporation. All rights reserved.					*/
/* Portions Copyright(c) 2020 Analog Devices Inc.								*/
/* Licensed under the MIT License.												*/
/*																				*/
/********************************************************************************/

#include "ADIView.h"
#include <GLFW/glfw3.h>
#include <chrono>
#include <iostream>
//#include <GL\gl3w.h>
#include <math.h>
#include <chrono>
#include <omp.h>

#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif

// [Win32] Our example includes a copy of glfw3.lib pre-compiled with VS2010 to
// maximize ease of testing and compatibility with old VS compilers. To link
// with VS2010-era libraries, VS2015+ requires linking with
// legacy_stdio_definitions.lib, which we do using this pragma. Your own project
// should not be affected, as you are likely to link with a newer binary of GLFW
// that is adequate for your version of Visual Studio.
#if defined(_MSC_VER) && (_MSC_VER >= 1900) &&                                 \
    !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif

#undef min
#undef max

using namespace adiviewer;
using namespace adicontroller;

ADIView::ADIView(std::shared_ptr<ADIController> ctrl, const std::string &name)
    : m_ctrl(ctrl), m_viewName(name), m_frameAvailable(false),
      m_center(true), m_waitKeyBarrier(0), m_distanceVal(0),
      m_smallSignal(false), m_crtSmallSignalState(false) {
    //Create AB and Depth independent threads

    ab_video_data_8bit = nullptr;
    depth_video_data_8bit = nullptr;
    m_abImageWorker = std::thread(std::bind(&ADIView::prepareImages, this));
}

ADIView::~ADIView() {
    if (m_ctrl->hasCamera()) {
        m_ctrl->StopCapture();
    }

    std::unique_lock<std::mutex> lock(m_frameCapturedMutex);
    m_stopWorkersFlag = true;
    lock.unlock();
    m_frameCapturedCv.notify_all();
    m_abImageWorker.join();

    if (ab_video_data_8bit != nullptr) {
        delete [] ab_video_data_8bit;
        ab_video_data_8bit = nullptr;
    }

    if (depth_video_data_8bit != nullptr) {
        delete[] depth_video_data_8bit;
        depth_video_data_8bit = nullptr;
    }

    if (normalized_vertices != nullptr) {
        delete[] normalized_vertices;
        normalized_vertices = nullptr;
    }
}

static void glfw_error_callback(int error, const char *description) {
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

void ADIView::cleanUp() {
	// What should this do now?
}

void ADIView::setABMaxRange(std::string value) {
    uint16_t base = 13; // Cap at 8191

    if (getCapABWidth() == false) {
        if (value == "6")
            base = 16;
        else if (value == "5")
            base = 14;
        else if (value == "4")
            base = 12;
        else if (value == "3")
            base = 10;
        else
            base = 8;
    }
    m_maxABPixelValue = (1 << base) - 1;
}

void ADIView::prepareImages() {
    while (!m_stopWorkersFlag) {

        {
            std::unique_lock<std::mutex> lock(m_frameCapturedMutex);
            m_frameCapturedCv.wait(lock, [&]() {
                return m_frameAvailable || m_stopWorkersFlag;
                });

            if (m_stopWorkersFlag) {
                break;
            }

            m_frameAvailable = false;
            if (m_capturedFrame == nullptr) {
                continue;
            }

            lock.unlock(); // Lock is no longer needed
        }

        auto start = std::chrono::high_resolution_clock::now();

		std::thread depthThread(&ADIView::_displayDepthImage, this);
		std::thread abThread(&ADIView::_displayAbImage, this);
        if (m_pccolour == 1) {
            abThread.join();
        }
		std::thread pointCloudThread(&ADIView::_displayPointCloudImage, this);
        if (m_pccolour != 1) {
            abThread.join();
        }
        depthThread.join();
		pointCloudThread.join();

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = end - start;
        LOG(INFO) << "Processing time: " << duration.count() << " ms";

        std::unique_lock<std::mutex> imshow_lock(m_imshowMutex);

        m_waitKeyBarrier = 3;

        if (m_waitKeyBarrier == numOfThreads) {
            imshow_lock.unlock();
            m_barrierCv.notify_one();
        }
    }

    // TODO: Still not sure this is the best place to do this
    LOG(INFO) << __func__ << " CleanUp";
    vertexArraySize = 0;
    m_capturedFrame = nullptr;
    ab_video_data = nullptr;
    depth_video_data = nullptr;
    pointCloud_video_data = nullptr;

    if (ab_video_data_8bit != nullptr) {
        delete[] ab_video_data_8bit;
        ab_video_data_8bit = nullptr;
    }

    if (depth_video_data_8bit != nullptr) {
        delete[] depth_video_data_8bit;
        depth_video_data_8bit = nullptr;
    }

    if (normalized_vertices != nullptr) {
        delete[] normalized_vertices;
        normalized_vertices = nullptr;
    }
}

void ADIView::_displayAbImage() {
    uint16_t* _ab_video_data = nullptr;
    
    auto camera = m_ctrl->m_cameras[static_cast<unsigned int>(
        m_ctrl->getCameraInUse())];

    m_capturedFrame->getData("ab", &ab_video_data);

    aditof::FrameDataDetails frameAbDetails;
    frameAbDetails.height = 0;
    frameAbDetails.width = 0;
    m_capturedFrame->getDataDetails("ab", frameAbDetails);

    frameHeight = static_cast<int>(frameAbDetails.height);
    frameWidth = static_cast<int>(frameAbDetails.width);

    // Update a copy of the AB frame buffer since the origina may be used by the recorder.
    if (_ab_video_data == nullptr) {
        _ab_video_data = new uint16_t[frameHeight * frameWidth];
    }

    if (_ab_video_data == nullptr) {
        LOG(ERROR) << __func__ << ": Cannot allocate _ab_video_data.";
        return;
    }

    memcpy(_ab_video_data, ab_video_data,
            frameHeight * frameWidth * sizeof(uint16_t));

    camera->normalizeABBuffer(_ab_video_data, frameWidth, frameHeight,
                                getAutoScale(), getLogImage());

    size_t imageSize = frameHeight * frameWidth;
    size_t bgrSize = 0;

    if (ab_video_data_8bit == nullptr) {
        ab_video_data_8bit = new uint8_t[frameHeight * frameWidth * 3];
    }

    for (int32_t dummyCtr = 0; dummyCtr < imageSize; dummyCtr++) {
        uint16_t pix = _ab_video_data[dummyCtr];
        ab_video_data_8bit[bgrSize++] = (uint8_t)(pix);
        ab_video_data_8bit[bgrSize++] = (uint8_t)(pix);
        ab_video_data_8bit[bgrSize++] = (uint8_t)(pix);
    }
    if (_ab_video_data != nullptr) {
        delete[] _ab_video_data;
    }
}

void ADIView::_displayDepthImage() {
    uint16_t *data;
    m_capturedFrame->getData("depth", &depth_video_data);

    aditof::FrameDataDetails frameDepthDetails;
    m_capturedFrame->getDataDetails("depth", frameDepthDetails);

    frameHeight = static_cast<int>(frameDepthDetails.height);
    frameWidth = static_cast<int>(frameDepthDetails.width);

    constexpr uint8_t PixelMax = std::numeric_limits<uint8_t>::max();
    size_t imageSize = frameHeight * frameWidth;
    size_t bgrSize = 0;

    if (depth_video_data_8bit == nullptr) {
        depth_video_data_8bit =
            new uint8_t[frameHeight * frameWidth * 3]; //Multiplied by BGR
    }

    float fRed = 0.f;
    float fGreen = 0.f;
    float fBlue = 0.f;
    //uint8_t blueErase;
    //uint8_t greenErase;
    //uint8_t redErase;

    //Create a dummy for loop that mimics some future process
    for (size_t dummyCtr = 0; dummyCtr < imageSize; dummyCtr++) {
        //It is doing a width x height x 3: Resolution * 3bytes (BGR)
        if (depth_video_data[dummyCtr] ==
            0) { //If pixel is actual zero, leave it as zero
            depth_video_data_8bit[bgrSize++] = 0;
            depth_video_data_8bit[bgrSize++] = 0;
            depth_video_data_8bit[bgrSize++] = 0;
        } else {
            hsvColorMap(depth_video_data[dummyCtr], maxRange, minRange,
                        fRed, fGreen, fBlue);
            depth_video_data_8bit[bgrSize++] =
                static_cast<uint8_t>(fBlue * PixelMax); //Blue
            depth_video_data_8bit[bgrSize++] =
                static_cast<uint8_t>(fGreen * PixelMax); //Green
            depth_video_data_8bit[bgrSize++] =
                static_cast<uint8_t>(fRed * PixelMax); //Red

            //blueErase = static_cast<uint8_t>(fBlue * PixelMax);
            //greenErase = static_cast<uint8_t>(fGreen * PixelMax);
            //redErase = static_cast<uint8_t>(fRed * PixelMax);
        }
    }
}

void ADIView::_displayPointCloudImage() {
    //Get XYZ table
    m_capturedFrame->getData("xyz", &pointCloud_video_data);
    aditof::FrameDataDetails frameXyzDetails;
    frameXyzDetails.height = 0;
    frameXyzDetails.width = 0;
    m_capturedFrame->getDataDetails("xyz", frameXyzDetails);
    frameHeight = static_cast<int>(frameXyzDetails.height);
    frameWidth = static_cast<int>(frameXyzDetails.width);

    //Size is [XX, YY, ZZ] x Width x Height
    size_t frameSize = frameHeight * frameWidth * 3;
    if (normalized_vertices == nullptr || pointcloudTableSize != frameSize) {
        if (normalized_vertices) {
            delete[] normalized_vertices;
        }
        pointcloudTableSize = frameSize;
        normalized_vertices =
            new float[pointcloudTableSize * 3]; //Adding RGB components
    }

    float fRed = 0.f;
    float fGreen = 0.f;
    float fBlue = 0.f;
    size_t bgrSize = 0;
    size_t cntr = 0;
    vertexArraySize = 0;

    constexpr uint8_t PixelMax = std::numeric_limits<uint8_t>::max();

    //1) convert the buffer from uint16 to float
    //2) normalize between [-1.0, 1.0]
    //3) X and Y ranges between [-32768, 32767] or [FFFF, 7FFF]. Z axis is [0, 7FFF]

    for (int i = 0; i < pointcloudTableSize; i += 3) {
            
        //XYZ
        normalized_vertices[bgrSize++] =
            (((int16_t)pointCloud_video_data[i])) / (Max_X);
        normalized_vertices[bgrSize++] =
            (((int16_t)pointCloud_video_data[i + 1])) / ((Max_Y)); 
        normalized_vertices[bgrSize++] =
            (((int16_t)pointCloud_video_data[i + 2])) / ((Max_Z));

        //RGB
        if ((int16_t)pointCloud_video_data[i + 2] == 0) {

            normalized_vertices[bgrSize++] = 0.0; //R = 0.0
            normalized_vertices[bgrSize++] = 0.0; //G = 0.0
            normalized_vertices[bgrSize++] = 0.0; //B = 0.0
            cntr += 3;

        } else {
            if (m_pccolour == 1) {
                normalized_vertices[bgrSize++] =
                    (float)ab_video_data_8bit[cntr++] / 255.0f;
                normalized_vertices[bgrSize++] =
                    (float)ab_video_data_8bit[cntr++] / 255.0f;
                normalized_vertices[bgrSize++] =
                    (float)ab_video_data_8bit[cntr++] / 255.0f;
            } else {
                hsvColorMap((pointCloud_video_data[i + 2]), maxRange, minRange,
                    fRed, fGreen, fBlue);
                normalized_vertices[bgrSize++] = fRed;
                normalized_vertices[bgrSize++] = fGreen;
                normalized_vertices[bgrSize++] = fBlue;
            }
        }
    }

    vertexArraySize =
        pointcloudTableSize * sizeof(float) * 3; //Adding RGB component
}

void ADIView::hsvColorMap(uint16_t video_data, int max, int min, float &fRed,
                          float &fGreen, float &fBlue) {
    int ClampedValue = video_data;
    ClampedValue = std::min(ClampedValue, max);
    ClampedValue = std::max(ClampedValue, min);
    float hue = 0;
    // The 'hue' coordinate in HSV is a polar coordinate, so it 'wraps'.
    // Purple starts after blue and is close enough to red to be a bit unclear,
    // so we want to go from blue to red.  Purple starts around .6666667,
    // so we want to normalize to [0, .6666667].
    //
    constexpr float range = 2.f / 3.f;
    // Normalize to [0, 1]
    //
    hue = (ClampedValue - min) / static_cast<float>(max - min);
    hue *= range;

    // We want blue to be close and red to be far, so we need to reflect the
    // hue across the middle of the range.
    //
    //hue = range - hue; // Disable HSV format. With this line commented blue is far and red is close.

    fRed = 0.f;
    fGreen = 0.f;
    fBlue = 0.f;
    ImGui::ColorConvertHSVtoRGB(hue, 1.f, 1.f, fRed, fGreen, fBlue);
}

/****************/
//OpenCV  ~deprecated
unsigned int texture;
void ADIView::startCamera() {
    // set up vertex data (and buffer(s)) and configure vertex attributes
    // ------------------------------------------------------------------
    float vertices[] = {
        // positions          // colors       // texture coords
        0.5f,  0.5f,  0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f, // top right
        0.5f,  -0.5f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, // bottom right
        -0.5f, -0.5f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, // bottom left
        -0.5f, 0.5f,  0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f  // top left
    };

    // Set up index
    unsigned int indices[] = {
        0, 1, 3, // first triangle
        1, 2, 3  // second triangle
    };

    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
}
