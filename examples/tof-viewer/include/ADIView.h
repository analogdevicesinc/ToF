/********************************************************************************/
/*                                                                              */
/* Copyright (c) Microsoft Corporation. All rights reserved.					*/
/*  Portions Copyright (c) 2020 Analog Devices Inc.								*/
/* Licensed under the MIT License.												*/
/*																				*/
/********************************************************************************/

#ifndef ADIVIEW_H
#define ADIVIEW_H

#include <fstream>
#include <stdio.h>

#include <iostream>
#include <deque>
#include <chrono>
#include <numeric>

#include "ADIController.h"
#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include <ADIShader.h>
#include <aditof/frame.h>

#ifdef __ARM_NEON or __ARM_NEON__

// TODO ARM NEON

#else

#define AB_SIMD /* Much faster, so leave this active */
#define DEPTH_SIMD /* Much faster, so leave this active */
//#define PC_SIMD

#endif //__ARM_NEON or __ARM_NEON__

#if defined(_WIN32) || defined(__WIN32__) || defined(WIN32)
//#define AB_TIME
//#define DEPTH_TIME
//#define PC_TIME
#endif //defined(_WIN32) || defined(__WIN32__) || defined(WIN32)

namespace adiviewer {
struct ImageDimensions {
    ImageDimensions() = default;
    constexpr ImageDimensions(int w, int h) : Width(w), Height(h) {}
    constexpr ImageDimensions(const std::pair<int, int> &pair)
        : Width(pair.first), Height(pair.second) {}

    int Width;
    int Height;
};

class ADIView {
  public:
    /**
		* @brief Constructor
		*/
    ADIView(std::shared_ptr<adicontroller::ADIController> ctrl,
            const std::string &name);

    /**
		* @brief Destructor
		*/
    ~ADIView();

    void cleanUp();

    /**
		* @brief Not implemented. Code under development
		*/
    bool startImGUI(bool *success);

    void setLogImage(bool value) { m_logImage = value; }
    bool getLogImage() { return m_logImage; }

    void setSaveBinaryFormat(bool value) { m_saveBinaryFormat = value; }
    bool getSaveBinaryFormat() { return m_saveBinaryFormat; }

    void setCapABWidth(bool value) { m_capABWidth = value; }
    bool getCapABWidth() { return m_capABWidth; }

    void setAutoScale(bool value) { m_autoScale = value; }
    bool getAutoScale() { return m_autoScale; }

    void setABMaxRange(std::string value);
    void setABMaxRange(uint32_t value) { m_maxABPixelValue = value; }
    uint32_t getABMaxRange() { return m_maxABPixelValue; }

    void setABMinRange(uint32_t value) { m_minABPixelValue = value; }
    uint32_t getABMinRange() { return m_minABPixelValue; }

    void setUserABMaxState(bool value) { m_maxABPixelValueSet = value; }
    bool getUserABMaxState() { return m_maxABPixelValueSet; }

    void setUserABMinState(bool value) { m_minABPixelValueSet = value; }
    bool getUserABMinState() { return m_minABPixelValueSet; }

    void setPointCloudColour(uint32_t colour) { m_pccolour = colour; }

    std::shared_ptr<adicontroller::ADIController> m_ctrl;
    std::shared_ptr<aditof::Frame> m_capturedFrame = nullptr;
    std::condition_variable m_barrierCv;
    std::mutex m_imshowMutex;
    uint32_t frameHeight = 0;
    uint32_t frameWidth = 0;
    int32_t m_waitKeyBarrier;
    uint32_t numOfThreads = 3;
    std::mutex m_frameCapturedMutex;
    bool m_abFrameAvailable;
    bool m_depthFrameAvailable;
    bool m_pcFrameAvailable;
    bool m_stopWorkersFlag = false;
    bool m_saveBinaryFormat = false;
    uint32_t m_pccolour = 0;

    std::thread m_depthImageWorker;
    std::thread m_abImageWorker;
    std::thread m_pointCloudImageWorker;
    std::condition_variable m_frameCapturedCv;
    uint16_t *ab_video_data;
    uint16_t *depth_video_data;
    int16_t *pointCloud_video_data;
    uint8_t *ab_video_data_8bit;
    uint8_t *depth_video_data_8bit;
    float *normalized_vertices = nullptr;
    size_t pointcloudTableSize = 0;

    uint16_t temperature_c;
    uint16_t time_stamp;
    double m_blendValue = 0.5;
    int32_t maxRange = 5000;
    int32_t minRange = 0;
    /**************/
    //OpenCV  here
    /**
		* @brief Deprecated
		*/
    void startCamera();

    //Point Cloud
    GLint viewIndex;
    GLint modelIndex;
    GLint projectionIndex;
    GLint m_pointSizeIndex;
    GLuint vertexArrayObject;
    GLuint vertexBufferObject; //Image Buffer
    adiviewer::Program pcShader;
    uint32_t vertexArraySize = 0;
    float Max_Z = 6000.0;
    float Min_Z = 0.0;
    float Max_Y = 6000.0;
    float Max_X = 6000.0;

  private:
    void prepareImages();
    /**
		* @brief Creates Depth buffer data
		*/
#ifdef DEPTH_SIMD
    void _displayDepthImage_SIMD();
#else //DEPTH_SIMD
    void _displayDepthImage();
#endif //DEPTH_SIMD

    /**
		* @brief Creates AB buffer data
		*/
#ifdef AB_SIMD
    void _displayAbImage_SIMD();
    void normalizeABBuffer_SIMD(uint16_t* abBuffer, uint16_t abWidth,
        uint16_t abHeight, bool advanceScaling,
        bool useLogScaling);
#else //AB_SIMD
    void _displayAbImage();
    void normalizeABBuffer(uint16_t* abBuffer, uint16_t abWidth,
        uint16_t abHeight, bool advanceScaling,
        bool useLogScaling);
#endif //AB_SIMD

    /**
		* @brief Creates a Point Cloud buffer data
		*/
#ifdef PC_SIMD
    void _displayPointCloudImage_SIMD();
#else //PC_SIMD
    void _displayPointCloudImage();
#endif //PC_SIMD

    /**
		* @brief Returns RGB components in
		*        HSV format
		*/
    void hsvColorMap(uint16_t video_data, int max, int min, float &fRed,
                     float &fGreen, float &fBlue);

    void ColorConvertHSVtoRGB(float h, float s, float v, float& out_r, float& out_g, float& out_b);
   

    std::string m_viewName;
    bool m_center;
    int m_distanceVal;
    bool m_smallSignal;
    bool m_crtSmallSignalState;

    //imGUI stuff
    GLFWwindow *window;
    bool showABWindow = true;
    bool showDepthWindow = true;
    bool beginDisplayABImage = false;
    bool beginDisplayDepthImage = false;
    bool beginDisplayPointCloudImage = false;

    uint16_t *video_data;
    const char *vertexShaderSource;
    const char *fragmentShaderSource;
    int shaderProgram;
    uint32_t m_maxABPixelValue;
    uint32_t m_minABPixelValue;
    bool m_maxABPixelValueSet = false;
    bool m_minABPixelValueSet = false;
    bool m_logImage = true;
    bool m_capABWidth = false;
    bool m_autoScale = true;

    std::mutex ab_data_ready_mtx;
    std::condition_variable ab_data_ready_cv;
    bool ab_data_ready = false;

    const size_t N = 50;

    // Call this before your function
    auto startTimer() {
		return std::chrono::high_resolution_clock::now();
    }

    // Call this after your function; updates 'times' and returns running average in ms
    double endTimerAndUpdate(std::chrono::time_point<std::chrono::high_resolution_clock> timerStart, std::deque<long long> *times) {
        auto end = std::chrono::high_resolution_clock::now();
        long long duration;
    
        duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - timerStart).count();
        
        times->push_back(duration);
        if (times->size() > N) times->pop_front();

        double sum = std::accumulate(times->begin(), times->end(), 0.0);
        return sum / times->size() / 1e6; // Average in milliseconds
    }
};
} //namespace adiviewer

#endif
