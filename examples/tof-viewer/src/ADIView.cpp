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

#include <sstream>


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
    : m_ctrl(ctrl), m_viewName(name),
      m_center(true), m_waitKeyBarrier(0), m_distanceVal(0),
      m_smallSignal(false), m_crtSmallSignalState(false) {


    m_depthFrameAvailable = false;
    m_pcFrameAvailable = false;
	m_abFrameAvailable = false;

    ab_video_data_8bit = nullptr;
    depth_video_data_8bit = nullptr;
    normalized_vertices = nullptr;

#ifdef DEPTH_SIMD
    m_depthImageWorker = std::thread(std::bind(&ADIView::_displayDepthImage_SIMD, this));
#else //DEPTH_SIMD
	m_depthImageWorker = std::thread(std::bind(&ADIView::_displayDepthImage, this));
#endif //DEPTH_SIMD

#ifdef AB_SIMD
    m_abImageWorker = std::thread(std::bind(&ADIView::_displayAbImage_SIMD, this));
#else //AB_SIMD
    m_abImageWorker = std::thread(std::bind(&ADIView::_displayAbImage, this));
#endif //AB_SIMD

#ifdef PC_SIMD
    m_pointCloudImageWorker = std::thread(std::bind(&ADIView::_displayPointCloudImage_SIMD, this));
#else //PC_SIMD
    m_pointCloudImageWorker = std::thread(std::bind(&ADIView::_displayPointCloudImage, this));
#endif //PC_SIMD
}

ADIView::~ADIView() {
    if (m_ctrl->hasCamera()) {
        m_ctrl->StopCapture();
    }

    std::unique_lock<std::mutex> lock(m_frameCapturedMutex);
    m_stopWorkersFlag = true;
    lock.unlock();
    m_frameCapturedCv.notify_all();

    m_depthImageWorker.join();
    m_abImageWorker.join();
    m_pointCloudImageWorker.join();

    vertexArraySize = 0;
    m_capturedFrame = nullptr;


    cleanUp();
}

void ADIView::cleanUp() {
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

#ifdef AB_SIMD
void ADIView::normalizeABBuffer_SIMD(
    uint16_t* abBuffer, uint16_t abWidth,
    uint16_t abHeight, bool advanceScaling,
    bool useLogScaling
) {
    size_t imageSize = abHeight * abWidth;
    uint32_t min_value_of_AB_pixel = 0xFFFF;
    uint32_t max_value_of_AB_pixel = 1;
    const size_t simd_width = 16; // AVX2: 16 x uint16_t

    // --- Min/Max scan (can be flat buffer, it's just a scan) ---
    if (advanceScaling) {
        __m256i vmin = _mm256_set1_epi16(0xFFFF);
        __m256i vmax = _mm256_setzero_si256();
        size_t i = 0;
        for (; i + simd_width <= imageSize; i += simd_width) {
            __m256i v = _mm256_loadu_si256((__m256i*)(abBuffer + i));
            vmin = _mm256_min_epu16(vmin, v);
            vmax = _mm256_max_epu16(vmax, v);
        }
        alignas(32) uint16_t min_buf[simd_width], max_buf[simd_width];
        _mm256_store_si256((__m256i*)min_buf, vmin);
        _mm256_store_si256((__m256i*)max_buf, vmax);
        for (int j = 0; j < simd_width; ++j) {
            min_value_of_AB_pixel = std::min(min_value_of_AB_pixel, (uint32_t)min_buf[j]);
            max_value_of_AB_pixel = std::max(max_value_of_AB_pixel, (uint32_t)max_buf[j]);
        }
        for (; i < imageSize; ++i) {
            min_value_of_AB_pixel = std::min(min_value_of_AB_pixel, (uint32_t)abBuffer[i]);
            max_value_of_AB_pixel = std::max(max_value_of_AB_pixel, (uint32_t)abBuffer[i]);
        }
        max_value_of_AB_pixel -= min_value_of_AB_pixel;
    }
    else {
        uint32_t m_maxABPixelValue = (1 << 13) - 1;
        max_value_of_AB_pixel = m_maxABPixelValue;
        min_value_of_AB_pixel = 0;
    }

    uint32_t new_max_value_of_AB_pixel = 1;
    uint32_t new_min_value_of_AB_pixel = 0xFFFF;

    // --- SIMD normalization: process row by row ---
    float norm_factor = 255.0f / float(max_value_of_AB_pixel);
    __m256 normV = _mm256_set1_ps(norm_factor);
    __m256i minV = _mm256_set1_epi16((short)min_value_of_AB_pixel);
    __m256i global_vmin = _mm256_set1_epi16(0xFFFF);
    __m256i global_vmax = _mm256_setzero_si256();

    for (uint16_t y = 0; y < abHeight; ++y) {
        size_t row_start = y * abWidth;
        size_t x = 0;
        for (; x + simd_width <= abWidth; x += simd_width) {
            __m256i pix = _mm256_loadu_si256((__m256i*)(abBuffer + row_start + x));
            __m256i pix_sub = _mm256_subs_epu16(pix, minV);

            // Convert to 32-bit int, then float
            __m128i lo = _mm256_extracti128_si256(pix_sub, 0);
            __m128i hi = _mm256_extracti128_si256(pix_sub, 1);

            __m256 flo = _mm256_cvtepi32_ps(_mm256_cvtepu16_epi32(lo));
            __m256 fhi = _mm256_cvtepi32_ps(_mm256_cvtepu16_epi32(hi));

            // Normalize
            flo = _mm256_mul_ps(flo, normV);
            fhi = _mm256_mul_ps(fhi, normV);

            // Clamp [0,255]
            flo = _mm256_max_ps(_mm256_setzero_ps(), _mm256_min_ps(flo, _mm256_set1_ps(255.0f)));
            fhi = _mm256_max_ps(_mm256_setzero_ps(), _mm256_min_ps(fhi, _mm256_set1_ps(255.0f)));

            // Convert back to int16
            __m256i ilo = _mm256_cvtps_epi32(flo);
            __m256i ihi = _mm256_cvtps_epi32(fhi);

            // Pack and store
            __m128i out_lo = _mm_packus_epi32(_mm256_castsi256_si128(ilo), _mm256_extracti128_si256(ilo, 1));
            __m128i out_hi = _mm_packus_epi32(_mm256_castsi256_si128(ihi), _mm256_extracti128_si256(ihi, 1));
            __m256i out = _mm256_set_m128i(out_hi, out_lo);

            _mm256_storeu_si256((__m256i*)(abBuffer + row_start + x), out);

            global_vmin = _mm256_min_epu16(global_vmin, out);
            global_vmax = _mm256_max_epu16(global_vmax, out);
        }
        // Scalar tail for this row
        for (; x < abWidth; ++x) {
            int val = int(abBuffer[row_start + x]) - int(min_value_of_AB_pixel);
            float pix = float(val) * norm_factor;
            if (pix < 0.0f) pix = 0.0f;
            if (pix > 255.0f) pix = 255.0f;
            abBuffer[row_start + x] = (uint8_t)pix;
        }
    }
    // Scalar reduction for new min/max
    alignas(32) uint16_t min_buf[simd_width], max_buf[simd_width];
    _mm256_store_si256((__m256i*)min_buf, global_vmin);
    _mm256_store_si256((__m256i*)max_buf, global_vmax);
    for (int j = 0; j < simd_width; ++j) {
        new_min_value_of_AB_pixel = std::min(new_min_value_of_AB_pixel, (uint32_t)min_buf[j]);
        new_max_value_of_AB_pixel = std::max(new_max_value_of_AB_pixel, (uint32_t)max_buf[j]);
    }
    for (size_t k = abHeight * abWidth - (abHeight * abWidth) % simd_width; k < abHeight * abWidth; ++k) {
        new_min_value_of_AB_pixel = std::min(new_min_value_of_AB_pixel, (uint32_t)abBuffer[k]);
        new_max_value_of_AB_pixel = std::max(new_max_value_of_AB_pixel, (uint32_t)abBuffer[k]);
    }

    // --- Log scaling, row-wise ---
    if (useLogScaling) {
        max_value_of_AB_pixel = new_max_value_of_AB_pixel;
        min_value_of_AB_pixel = new_min_value_of_AB_pixel;
        double maxLogVal = log10(1.0 + double(max_value_of_AB_pixel - min_value_of_AB_pixel));
        const size_t log_simd_width = 8;
        __m256 minVf = _mm256_set1_ps(float(min_value_of_AB_pixel));
        __m256 maxLogValV = _mm256_set1_ps(float(maxLogVal));
        for (uint16_t y = 0; y < abHeight; ++y) {
            size_t row_start = y * abWidth;
            size_t x = 0;
            for (; x + log_simd_width <= abWidth; x += log_simd_width) {
                // Load 8 uint16
                __m128i v = _mm_loadu_si128((__m128i*)(abBuffer + row_start + x));
                __m256i v32 = _mm256_cvtepu16_epi32(v);
                __m256 vf = _mm256_cvtepi32_ps(v32);

                // log10(1.0 + pix - min)
                vf = _mm256_sub_ps(vf, minVf);
                vf = _mm256_add_ps(vf, _mm256_set1_ps(1.0f));
                alignas(32) float buf[log_simd_width];
                _mm256_store_ps(buf, vf);
                for (int j = 0; j < log_simd_width; ++j) {
                    buf[j] = std::log10(buf[j]);
                }
                vf = _mm256_load_ps(buf);

                // pix = (logPix / maxLogVal) * 255.0
                vf = _mm256_div_ps(vf, maxLogValV);
                vf = _mm256_mul_ps(vf, _mm256_set1_ps(255.0f));
                vf = _mm256_max_ps(_mm256_setzero_ps(), _mm256_min_ps(vf, _mm256_set1_ps(255.0f)));

                // Store back as uint8
                __m256i vi = _mm256_cvtps_epi32(vf);
                __m128i out = _mm_packus_epi32(_mm256_castsi256_si128(vi), _mm256_extracti128_si256(vi, 1));
                for (int j = 0; j < 8; ++j) {
                    abBuffer[row_start + x + j] = uint8_t(((uint16_t*)&out)[j]);
                }
            }
            // Remainder for row
            for (; x < abWidth; ++x) {
                double pix = double(abBuffer[row_start + x]) - min_value_of_AB_pixel;
                double logPix = log10(1.0 + pix);
                pix = (logPix / maxLogVal) * 255.0;
                if (pix < 0.0) pix = 0.0;
                if (pix > 255.0) pix = 255.0;
                abBuffer[row_start + x] = (uint8_t)pix;
            }
        }
    }
}

// --- Display AB image with row-wise SIMD BGR packing ---
void ADIView::_displayAbImage_SIMD() {

#ifdef AB_TIME
    std::deque<long long> timeABQ;
#endif //AB_TIME

    while (!m_stopWorkersFlag) {

        {
            std::unique_lock<std::mutex> lock(m_frameCapturedMutex);
            m_frameCapturedCv.wait(lock, [&]() {
                return m_abFrameAvailable || m_stopWorkersFlag;
                });

            if (m_stopWorkersFlag) {
                break;
            }

            m_abFrameAvailable = false;
            if (m_capturedFrame == nullptr) {
                continue;
            }

            lock.unlock(); // Lock is no longer needed
        }
#ifdef AB_TIME
        auto timerStart = startTimer();
#endif //AB_TIME

        std::lock_guard<std::mutex> lock(ab_data_ready_mtx);

        uint16_t* _ab_video_data = nullptr;
        auto camera = m_ctrl->m_cameras[static_cast<unsigned int>(m_ctrl->getCameraInUse())];

        m_capturedFrame->getData("ab", &ab_video_data);

        if (ab_video_data == nullptr) {
            return;
        }

        aditof::FrameDataDetails frameAbDetails;
        frameAbDetails.height = 0;
        frameAbDetails.width = 0;
        m_capturedFrame->getDataDetails("ab", frameAbDetails);

        frameHeight = static_cast<int>(frameAbDetails.height);
        frameWidth = static_cast<int>(frameAbDetails.width);

        if (_ab_video_data == nullptr) {
            _ab_video_data = new uint16_t[frameHeight * frameWidth];
        }
        if (_ab_video_data == nullptr) {
            LOG(ERROR) << __func__ << ": Cannot allocate _ab_video_data.";
            return;
        }
        memcpy(_ab_video_data, ab_video_data, frameHeight * frameWidth * sizeof(uint16_t));

        normalizeABBuffer_SIMD(_ab_video_data, frameWidth, frameHeight, getAutoScale(), getLogImage());

        size_t imageSize = frameHeight * frameWidth;
        size_t bgrSize = 0;

        if (ab_video_data_8bit == nullptr) {
            ab_video_data_8bit = new uint8_t[frameHeight * frameWidth * 3];
        }

        const size_t simd_width = 16;
        bgrSize = 0;
        for (int y = 0; y < frameHeight; ++y) {
            size_t row_start = y * frameWidth;
            size_t x = 0;
            for (; x + simd_width <= frameWidth; x += simd_width) {
                __m128i v_lo = _mm_loadu_si128((__m128i*)(_ab_video_data + row_start + x));
                __m128i v_hi = _mm_loadu_si128((__m128i*)(_ab_video_data + row_start + x + 8));
                __m128i v8 = _mm_packus_epi16(v_lo, v_hi);
                uint8_t buf[simd_width];
                _mm_storeu_si128((__m128i*)buf, v8);
                for (int j = 0; j < simd_width; ++j) {
                    uint8_t pix = buf[j];
                    ab_video_data_8bit[bgrSize++] = pix;
                    ab_video_data_8bit[bgrSize++] = pix;
                    ab_video_data_8bit[bgrSize++] = pix;
                }
            }
            for (; x < frameWidth; ++x) {
                uint8_t pix = (uint8_t)_ab_video_data[row_start + x];
                ab_video_data_8bit[bgrSize++] = pix;
                ab_video_data_8bit[bgrSize++] = pix;
                ab_video_data_8bit[bgrSize++] = pix;
            }
        }
        ab_data_ready = true;
        ab_data_ready_cv.notify_one();

        if (_ab_video_data != nullptr) {
            delete[] _ab_video_data;
        }

#ifdef AB_TIME
        //LOG(INFO) << "AB: " << endTimerAndUpdate(timerStart, &timeABQ) << " ms";
        {
            std::ostringstream oss;
            oss << "AB: " << endTimerAndUpdate(timerStart, &timeABQ) << " ms" << std::endl;
            OutputDebugStringA(oss.str().c_str());
        }
#endif //AB_TIME

        std::unique_lock<std::mutex> imshow_lock(m_imshowMutex);

        m_waitKeyBarrier++;

        if (m_waitKeyBarrier == numOfThreads) {
            imshow_lock.unlock();
            m_barrierCv.notify_one();
        }
    }
}
#else //AB_SIMD
void ADIView::normalizeABBuffer(uint16_t* abBuffer, uint16_t abWidth,
    uint16_t abHeight, bool advanceScaling,
    bool useLogScaling) {

    size_t imageSize = abHeight * abWidth;

    uint32_t min_value_of_AB_pixel = 0xFFFF;
    uint32_t max_value_of_AB_pixel = 1;

    if (advanceScaling) {

        for (size_t dummyCtr = 0; dummyCtr < imageSize; ++dummyCtr) {
            if (abBuffer[dummyCtr] > max_value_of_AB_pixel) {
                max_value_of_AB_pixel = abBuffer[dummyCtr];
            }
            if (abBuffer[dummyCtr] < min_value_of_AB_pixel) {
                min_value_of_AB_pixel = abBuffer[dummyCtr];
            }
        }
        max_value_of_AB_pixel -= min_value_of_AB_pixel;
    }
    else {

        //TODO: This is hard code, but should reflect the number of AB bits
        //      See https://github.com/analogdevicesinc/ToF/blob/7d63e2d7e0e2bb795f5a55139740b4d5870ad3d6/examples/tof-viewer/src/ADIView.cpp#L254
        uint32_t m_maxABPixelValue = (1 << 13) - 1;
        max_value_of_AB_pixel = m_maxABPixelValue;
        min_value_of_AB_pixel = 0;
    }

    uint32_t new_max_value_of_AB_pixel = 1;
    uint32_t new_min_value_of_AB_pixel = 0xFFFF;

    for (size_t dummyCtr = 0; dummyCtr < imageSize; ++dummyCtr) {

        abBuffer[dummyCtr] -= min_value_of_AB_pixel;
        double pix = abBuffer[dummyCtr] * (255.0 / max_value_of_AB_pixel);

        if (pix < 0.0) {
            pix = 0.0;
        }
        if (pix > 255.0) {
            pix = 255.0;
        }
        abBuffer[dummyCtr] = static_cast<uint8_t>(pix);

        if (abBuffer[dummyCtr] > new_max_value_of_AB_pixel) {
            new_max_value_of_AB_pixel = abBuffer[dummyCtr];
        }
        if (abBuffer[dummyCtr] < new_min_value_of_AB_pixel) {
            new_min_value_of_AB_pixel = abBuffer[dummyCtr];
        }
    }

    if (useLogScaling) {

        max_value_of_AB_pixel = new_max_value_of_AB_pixel;
        min_value_of_AB_pixel = new_min_value_of_AB_pixel;

        double maxLogVal =
            log10(1.0 + static_cast<double>(max_value_of_AB_pixel -
                min_value_of_AB_pixel));

        for (size_t dummyCtr = 0; dummyCtr < imageSize; ++dummyCtr) {

            double pix =
                static_cast<double>(abBuffer[dummyCtr] - min_value_of_AB_pixel);
            double logPix = log10(1.0 + pix);
            pix = (logPix / maxLogVal) * 255.0;
            if (pix < 0.0) {
                pix = 0.0;
            }
            if (pix > 255.0) {
                pix = 255.0;
            }

            abBuffer[dummyCtr] = (uint8_t)(pix);
        }
    }
}

void ADIView::_displayAbImage() {

#ifdef AB_TIME
    std::deque<long long> timeABQ;
#endif //AB_TIME

    while (!m_stopWorkersFlag) {

        //OutputDebugString("_displayAbImage: Wait\n");
        {
            std::unique_lock<std::mutex> lock(m_frameCapturedMutex);
            m_frameCapturedCv.wait(lock, [&]() {
                return m_abFrameAvailable || m_stopWorkersFlag;
                });

            if (m_stopWorkersFlag) {
                break;
            }

            m_abFrameAvailable = false;
            if (m_capturedFrame == nullptr) {
                continue;
            }

        lock.unlock(); // Lock is no longer needed

        auto camera = m_ctrl->m_cameras[static_cast<unsigned int>(
            m_ctrl->getCameraInUse())];

        m_capturedFrame->getData("ab", &ab_video_data);

        if (ab_video_data == nullptr) {
            //OutputDebugString("_displayAbImage: Error 1\n");
            return;
        }

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
            //OutputDebugString("_displayAbImage: Error 2\n");
            return;
        }

        memcpy(_ab_video_data, ab_video_data,
            frameHeight * frameWidth * sizeof(uint16_t));

        normalizeABBuffer(_ab_video_data, frameWidth, frameHeight,
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

        ab_data_ready = true;
        ab_data_ready_cv.notify_one();

        if (_ab_video_data != nullptr) {
            delete[] _ab_video_data;
        }

#ifdef AB_TIME
        //LOG(INFO) << "AB: " << endTimerAndUpdate(timerStart, &timeABQ) << " ms";
        {
            std::ostringstream oss;
            oss << "AB: " << endTimerAndUpdate(timerStart, &timeABQ) << " ms" << std::endl;
            OutputDebugStringA(oss.str().c_str());
        }
#endif //AB_TIME

        std::unique_lock<std::mutex> imshow_lock(m_imshowMutex);

        m_waitKeyBarrier++;

        if (m_waitKeyBarrier == numOfThreads) {
            imshow_lock.unlock();
            m_barrierCv.notify_one();
        }
    }
    //OutputDebugString("_displayAbImage: Exit\n");
}
#endif //AB_SIMD

#ifdef DEPTH_SIMD
void ADIView::_displayDepthImage_SIMD() {

#ifdef DEPTH_TIME
    std::deque<long long> timeDepthQ;
#endif //DEPTH_TIME

    while (!m_stopWorkersFlag) {

        {
            std::unique_lock<std::mutex> lock(m_frameCapturedMutex);
            m_frameCapturedCv.wait(lock, [&]() {
                return m_depthFrameAvailable || m_stopWorkersFlag;
                });

            if (m_stopWorkersFlag) {
                break;
            }

            m_depthFrameAvailable = false;
            if (m_capturedFrame == nullptr) {
                continue;
            }

            lock.unlock(); // Lock is no longer needed
        }
#ifdef DEPTH_TIME
        auto timerStart = startTimer();
#endif //DEPTH_TIME

        uint16_t* data;
        m_capturedFrame->getData("depth", &depth_video_data);

        if (depth_video_data == nullptr) {
            return;
        }

        aditof::FrameDataDetails frameDepthDetails;
        m_capturedFrame->getDataDetails("depth", frameDepthDetails);

        int frameHeight = static_cast<int>(frameDepthDetails.height);
        int frameWidth = static_cast<int>(frameDepthDetails.width);

        constexpr uint8_t PixelMax = std::numeric_limits<uint8_t>::max();
        size_t imageSize = frameHeight * frameWidth;
        size_t bgrSize = 0;

        if (depth_video_data_8bit == nullptr) {
            depth_video_data_8bit =
                new uint8_t[frameHeight * frameWidth * 3]; // BGR
        }

        constexpr int PixelBlock = 8; // 256 bits / 32 bits per float = 8, or 16 for 16bit ints
        size_t i = 0;

        // SIMD constants
        __m256 minRangeV = _mm256_set1_ps((float)minRange);
        __m256 maxRangeV = _mm256_set1_ps((float)maxRange);
        __m256 rangeV = _mm256_set1_ps((2.f / 3.f) / (maxRange - minRange));
        __m256 oneV = _mm256_set1_ps(1.f);

        for (; i + PixelBlock <= imageSize; i += PixelBlock) {
            // Load 8 uint16_t values and convert to float
            __m128i vSrc16 = _mm_loadu_si128((__m128i*) & depth_video_data[i]);
            // Zero extend to 32 bits
            __m256i vSrc32 = _mm256_cvtepu16_epi32(vSrc16);
            __m256 vSrcF = _mm256_cvtepi32_ps(vSrc32);

            // Compute mask for zero pixels
            __m256i zeroMask = _mm256_cmpeq_epi32(vSrc32, _mm256_setzero_si256());
            int mask = _mm256_movemask_epi8(zeroMask);

            // Clamp: max(min(x, max), min)
            __m256 vClamped = _mm256_max_ps(_mm256_min_ps(vSrcF, maxRangeV), minRangeV);

            // Normalize & scale hue
            __m256 vHue = _mm256_mul_ps(_mm256_sub_ps(vClamped, minRangeV), rangeV);

            // Scalar HSV->RGB, but could be vectorized for real performance
            float hues[PixelBlock];
            _mm256_storeu_ps(hues, vHue);

            for (int j = 0; j < PixelBlock; ++j) {
                size_t idx = i + j;
                if (depth_video_data[idx] == 0) {
                    depth_video_data_8bit[bgrSize++] = 0;
                    depth_video_data_8bit[bgrSize++] = 0;
                    depth_video_data_8bit[bgrSize++] = 0;
                }
                else {
                    float fRed, fGreen, fBlue;
                    ColorConvertHSVtoRGB(hues[j], 1.f, 1.f, fRed, fGreen, fBlue);
                    depth_video_data_8bit[bgrSize++] = static_cast<uint8_t>(fBlue * PixelMax);
                    depth_video_data_8bit[bgrSize++] = static_cast<uint8_t>(fGreen * PixelMax);
                    depth_video_data_8bit[bgrSize++] = static_cast<uint8_t>(fRed * PixelMax);
                }
            }
        }

        // Tail loop for leftovers
        for (; i < imageSize; ++i) {
            if (depth_video_data[i] == 0) {
                depth_video_data_8bit[bgrSize++] = 0;
                depth_video_data_8bit[bgrSize++] = 0;
                depth_video_data_8bit[bgrSize++] = 0;
            }
            else {
                float fRed, fGreen, fBlue;
                hsvColorMap(depth_video_data[i], maxRange, minRange, fRed, fGreen, fBlue);
                depth_video_data_8bit[bgrSize++] = static_cast<uint8_t>(fBlue * PixelMax);
                depth_video_data_8bit[bgrSize++] = static_cast<uint8_t>(fGreen * PixelMax);
                depth_video_data_8bit[bgrSize++] = static_cast<uint8_t>(fRed * PixelMax);
            }
        }

#ifdef DEPTH_TIME
        //LOG(INFO) << "Depth: " << endTimerAndUpdate(timerStart, &timeDepthQ) << " ms";
        {
            std::ostringstream oss;
            oss << "Depth: " << endTimerAndUpdate(timerStart, &timeDepthQ) << " ms" << std::endl;
            OutputDebugStringA(oss.str().c_str());
        }
#endif //DEPTH_TIME

        std::unique_lock<std::mutex> imshow_lock(m_imshowMutex);

        m_waitKeyBarrier++;

        if (m_waitKeyBarrier == numOfThreads) {
            imshow_lock.unlock();
            m_barrierCv.notify_one();
        }
    }
}

#else //DEPTH_SIMD

void ADIView::_displayDepthImage() {

#ifdef DEPTH_TIME
    std::deque<long long> timeDepthQ;
#endif //DEPTH_TIME

    while (!m_stopWorkersFlag) {

        //OutputDebugString("_displayDepthImage: Wait\n");
        {
            std::unique_lock<std::mutex> lock(m_frameCapturedMutex);
            m_frameCapturedCv.wait(lock, [&]() {
                return m_depthFrameAvailable || m_stopWorkersFlag;
                });

            if (m_stopWorkersFlag) {
                break;
            }

            m_depthFrameAvailable = false;
            if (m_capturedFrame == nullptr) {
                continue;
            }

            lock.unlock(); // Lock is no longer needed
        }
        //OutputDebugString("_displayDepthImage: Processing\n");

#ifdef DEPTH_TIME
        auto timerStart = startTimer();
#endif //DEPTH_TIME

        uint16_t* data;
        m_capturedFrame->getData("depth", &depth_video_data);

        if (depth_video_data == nullptr) {
            //OutputDebugString("_displayDepthImage: Error 1\n");
            return;
        }

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

        //Create a dummy for loop that mimics some future process
        for (size_t dummyCtr = 0; dummyCtr < imageSize; dummyCtr++) {
            //It is doing a width x height x 3: Resolution * 3bytes (BGR)
            if (depth_video_data[dummyCtr] ==
                0) { //If pixel is actual zero, leave it as zero
                depth_video_data_8bit[bgrSize++] = 0;
                depth_video_data_8bit[bgrSize++] = 0;
                depth_video_data_8bit[bgrSize++] = 0;
            }
            else {
                hsvColorMap(depth_video_data[dummyCtr], maxRange, minRange,
                    fRed, fGreen, fBlue);
                depth_video_data_8bit[bgrSize++] =
                    static_cast<uint8_t>(fBlue * PixelMax); //Blue
                depth_video_data_8bit[bgrSize++] =
                    static_cast<uint8_t>(fGreen * PixelMax); //Green
                depth_video_data_8bit[bgrSize++] =
                    static_cast<uint8_t>(fRed * PixelMax); //Red
            }
        }
#ifdef DEPTH_TIME
        //LOG(INFO) << "Depth: " << endTimerAndUpdate(timerStart, &timeDepthQ) << " ms";
        {
            std::ostringstream oss;
            oss << "Depth: " << endTimerAndUpdate(timerStart, &timeDepthQ) << " ms" << std::endl;
            OutputDebugStringA(oss.str().c_str());
        }
#endif //DEPTH_TIME

        std::unique_lock<std::mutex> imshow_lock(m_imshowMutex);

        m_waitKeyBarrier++;

        if (m_waitKeyBarrier == numOfThreads) {
            imshow_lock.unlock();
            m_barrierCv.notify_one();
        }
    }
    //OutputDebugString("_displayDepthImage: Exit\n");
}
#endif //DEPTH_SIMD

#ifdef PC_SIMD


#else //PC_SIMD
void ADIView::_displayPointCloudImage() {

#ifdef PC_TIME
    std::deque<long long> timePCQ;
#endif //AB_TIME

    while (!m_stopWorkersFlag) {

        {
            std::unique_lock<std::mutex> lock(m_frameCapturedMutex);
            m_frameCapturedCv.wait(lock, [&]() {
                return m_pcFrameAvailable || m_stopWorkersFlag;
                });

            if (m_stopWorkersFlag) {
                break;
            }

            m_pcFrameAvailable = false;
            if (m_capturedFrame == nullptr) {
                continue;
            }

            lock.unlock(); // Lock is no longer needed
        }

#ifdef PC_TIME
        auto timerStart = startTimer();
#endif //PC_TIME

        //Get XYZ table
        m_capturedFrame->getData("xyz", (uint16_t**)&pointCloud_video_data);

        if (pointCloud_video_data == nullptr) {
            return;
        }

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
                new float[(pointcloudTableSize + 1) * 3]; //Adding RGB components
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

        bool haveAb = m_capturedFrame->haveDataType("ab");

        if (haveAb) {
            std::unique_lock<std::mutex> lock(ab_data_ready_mtx);
            ab_data_ready_cv.wait(lock, [this] { return ab_data_ready; });  // wait for signal 
        }

        for (uint32_t i = 0; i < pointcloudTableSize; i += 3) {

            //XYZ
            normalized_vertices[bgrSize++] = static_cast<float>(pointCloud_video_data[i]) / Max_X;
            normalized_vertices[bgrSize++] = static_cast<float>(pointCloud_video_data[i + 1]) / Max_Y;
            normalized_vertices[bgrSize++] = static_cast<float>(pointCloud_video_data[i + 2]) / Max_Z;

            //RGB
            if ((int16_t)pointCloud_video_data[i + 2] == 0) {
                fRed = fGreen = fBlue = 0.0f;
                if (m_pccolour == 1) {
                    cntr += 3; // For 'm_pccolour == 1' case.
                }
            }
            else {
                if (m_pccolour == 2) {
                    fRed = fGreen = fBlue = 1.0f; //Default RGB values
                }
                else if (m_pccolour == 1 && haveAb) {
                    fRed = (float)ab_video_data_8bit[cntr] / 255.0f;
                    fGreen = (float)ab_video_data_8bit[cntr + 1] / 255.0f;
                    fBlue = (float)ab_video_data_8bit[cntr + 2] / 255.0f;
                    cntr += 3;
                }
                else {
                    hsvColorMap((pointCloud_video_data[i + 2]), maxRange, minRange,
                        fRed, fGreen, fBlue);
                }
            }
            normalized_vertices[bgrSize++] = fRed;
            normalized_vertices[bgrSize++] = fGreen;
            normalized_vertices[bgrSize++] = fBlue;
        }

        normalized_vertices[bgrSize++] = 0.0f; // X
        normalized_vertices[bgrSize++] = 0.0f; // Y
        normalized_vertices[bgrSize++] = 0.0f; // Z
        normalized_vertices[bgrSize++] = 1.0f; // R
        normalized_vertices[bgrSize++] = 1.0f; // G
        normalized_vertices[bgrSize++] = 1.0f; // 

        vertexArraySize =
            (pointcloudTableSize + 1) * sizeof(float) * 3; //Adding RGB component

#ifdef PC_TIME
        LOG(INFO) << "PC: " << endTimerAndUpdate(timerStart, &timePCQ) << " ms";
        {
            std::ostringstream oss;
            oss << "PC: " << endTimerAndUpdate(timerStart, &timePCQ) << " ms" << std::endl;
            OutputDebugStringA(oss.str().c_str());
        }
#endif //DEPTH_TIME

        std::unique_lock<std::mutex> imshow_lock(m_imshowMutex);

        m_waitKeyBarrier++;

        if (m_waitKeyBarrier == numOfThreads) {
            imshow_lock.unlock();
            m_barrierCv.notify_one();
        }
    }
}

#endif //PC_SIMD


void ADIView::ColorConvertHSVtoRGB(float h, float s, float v, float& out_r, float& out_g, float& out_b)
{
    if (s == 0.0f)
    {
        // gray
        out_r = out_g = out_b = v;
        return;
    }

    h = fmodf(h, 1.0f) / (60.0f / 360.0f);
    int   i = (int)h;
    float f = h - (float)i;
    float p = v * (1.0f - s);
    float q = v * (1.0f - s * f);
    float t = v * (1.0f - s * (1.0f - f));

    switch (i)
    {
    case 0: out_r = v; out_g = t; out_b = p; break;
    case 1: out_r = q; out_g = v; out_b = p; break;
    case 2: out_r = p; out_g = v; out_b = t; break;
    case 3: out_r = p; out_g = q; out_b = v; break;
    case 4: out_r = t; out_g = p; out_b = v; break;
    case 5: default: out_r = v; out_g = p; out_b = q; break;
    }
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
    ColorConvertHSVtoRGB(hue, 1.f, 1.f, fRed, fGreen, fBlue);
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
