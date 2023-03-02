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

#include "ADIController.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <ADIShader.h>
#include <aditof/frame.h>

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
    ADIView(std::shared_ptr<adicontroller::ADIController> &ctrl,
            const std::string &name);

    /**
		* @brief Destructor
		*/
    ~ADIView();

    /**
		* @brief Not implemented. Code under development
		*/
    void render();
    bool startImGUI(bool *success);

    /**
		* @brief Not implemented
		*/
    void showWindows();

    /**
		* @brief Not implemented
		*/
    void initVideo();

    /**
		* @brief Not implemented
		*/
    void captureVideo();

    /**
		* @brief Not implemented
		*/
    void initShaders();

    /**
		* @brief Will Try to render the image pixel by pixel
		*/
    uint8_t *imageRender(uint16_t *image);

    void setLogImage(bool value) { m_logImage = value; }
    bool getLogImage() { return m_logImage; }

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

    std::shared_ptr<adicontroller::ADIController> m_ctrl;
    std::shared_ptr<aditof::Frame> m_capturedFrame = nullptr;
    std::condition_variable m_barrierCv;
    std::mutex m_imshowMutex;
    int frameHeight = 0;
    int frameWidth = 0;
    int m_waitKeyBarrier;
    int numOfThreads = 2;
    std::mutex m_frameCapturedMutex;
    bool m_depthFrameAvailable;
    bool m_irFrameAvailable;
    bool m_pointCloudFrameAvailable;
    bool m_stopWorkersFlag = false;
    std::thread m_depthImageWorker;
    std::thread m_irImageWorker;
    std::thread m_pointCloudImageWorker;
    std::condition_variable m_frameCapturedCv;
    uint16_t *ir_video_data;
    uint16_t *depth_video_data;
    uint16_t *pointCloud_video_data;
    uint8_t *ir_video_data_8bit;
    uint8_t *depth_video_data_8bit;
    float *normalized_vertices = nullptr;
    size_t pointcloudTableSize = 0;

    unsigned short temperature_c;
    unsigned short time_stamp;
    double m_blendValue = 0.5;
    int maxRange = 5000;
    int minRange = 0;
    /**************/
    //OpenCV  here
    /**
		* @brief Deprecated
		*/
    void startCamera();

    //Point Cloud
    unsigned int viewIndex;
    unsigned int modelIndex;
    unsigned int projectionIndex;
    unsigned int vertexArrayObject;
    unsigned int vertexBufferObject; //Image Buffer
    adiviewer::Program pcShader;
    size_t vertexArraySize = 0;
    float Max_Z = 6000.0;
    float Min_Z = 0.0;
    float Max_Y = 6000.0;
    float Max_X = 6000.0;

  private:
    /**
		* @brief Creates Depth buffer data
		*/
    void _displayDepthImage();

    /**
		* @brief Creates IR buffer data
		*/
    void _displayIrImage();

    /**
		* @brief Creates a blended IR and Depth buffer data
		*/
    void _displayBlendedImage();

    /**
		* @brief Creates a Point Cloud buffer data
		*/
    void _displayPointCloudImage();

    /**
		* @brief Returns RGB components in
		*        HSV format
		*/
    void hsvColorMap(uint16_t video_data, int max, int min, float &fRed,
                     float &fGreen, float &fBlue);

    std::string m_viewName;
    bool m_center;
    int m_distanceVal;
    bool m_smallSignal;
    bool m_crtSmallSignalState;

    //imGUI stuff
    GLFWwindow *window;
    bool showIRWindow = true;
    bool showDepthWindow = true;
    bool beginDisplayIRImage = false;
    bool beginDisplayDepthImage = false;
    bool beginDisplayPointCloudImage = false;

    uint16_t *video_data;
    unsigned int video_texture = 0;
    bool needsInit = true;
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
};
} //namespace adiviewer

#endif
