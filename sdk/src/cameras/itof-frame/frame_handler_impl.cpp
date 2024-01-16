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

#include "frame_handler_impl.h"
#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif
using namespace aditof;

FrameHandlerImpl::FrameHandlerImpl()
    : m_concatFrames(true), m_enableMultithreading(false),
      m_customFormat(false), m_bitsInDepth(0), m_bitsInAB(0), m_bitsInConf(0),
      m_frameWidth(0), m_frameHeight(0), m_frameIndex(0), m_fileCreated(false),
      m_endOfFile(false), m_filePath("./"), m_pos(0), m_threadRunning(false) {}

FrameHandlerImpl::~FrameHandlerImpl() { m_threadWorker.join(); }

Status FrameHandlerImpl::setOutputFilePath(std::string &filePath) {
    Status status = Status::OK;
    m_filePath = filePath;
    m_fileCreated = false;
    return status;
}

Status FrameHandlerImpl::setInputFileName(std::string &fullFileName) {
    Status status = Status::OK;
    m_fullInputFileName = fullFileName;
    m_pos = 0;
    return status;
}

Status FrameHandlerImpl::saveFrameToFile(aditof::Frame &frame,
                                         std::string fileName) {
    Status status = Status::OK;

    if (m_concatFrames) {
        if (!m_fileCreated) {
            status = createFile(fileName);
        } else {
            m_file = std::fstream(m_fullOutputFileName,
                                  std::ios::app | std::ios::binary);
            m_file.seekg(std::ios::end);
        }
    } else {
        status = createFile(fileName);
    }

    if (status != Status::OK) {
        LOG(ERROR) << "Failed to create file!";
        return status;
    }

    //Store frames in file in followind order: metadata depth ab conf
    uint16_t *metaData;
    uint16_t *depthData;
    uint16_t *abData;
    uint16_t *confData;

    frame.getData("metadata", &metaData);
    frame.getData("depth", &depthData);
    frame.getData("ab", &abData);
    frame.getData("conf", &confData);

    Metadata metadataStruct;
    frame.getMetadataStruct(metadataStruct);

    //at first we assume that we have metadata enabled by default
    //TO DO: implement use-case where we don't have metadata
    m_file.write(reinterpret_cast<char *>(metaData), METADATA_SIZE);

    if (metadataStruct.bitsInDepth)
        m_file.write(reinterpret_cast<char *>(depthData),
                     metadataStruct.width * metadataStruct.height * 2);
    if (metadataStruct.bitsInAb)
        m_file.write(reinterpret_cast<char *>(abData),
                     metadataStruct.width * metadataStruct.height * 2);
    if (metadataStruct.bitsInConfidence)
        m_file.write(reinterpret_cast<char *>(confData),
                     metadataStruct.width * metadataStruct.height * 2);

    m_file.close();

    if (!m_frameQueue.empty()) {
        m_mutex.lock();
        m_frameQueue.pop();
        m_frameNameQueue.pop();
        m_mutex.unlock();
    }

    return status;
}

Status FrameHandlerImpl::saveFrameToFileMultithread(aditof::Frame &frame,
                                                    std::string fileName) {

    using namespace aditof;
    Status status = Status::OK;

    m_mutex.lock();
    m_frameQueue.push(std::move(frame));
    m_frameNameQueue.push(fileName);
    m_mutex.unlock();

    frame = Frame();

    if (!m_threadRunning) {
        if (m_threadWorker.joinable()) {
            m_threadWorker.join();
        }

        m_threadWorker =
            std::thread(std::bind(&FrameHandlerImpl::threadWritter, this));
        m_threadRunning = true;
    }

    return status;
}

void FrameHandlerImpl::threadWritter() {
    m_threadRunning = true;
    aditof::Status status;

    while (!m_frameQueue.empty()) {
        status =
            saveFrameToFile(m_frameQueue.front(), m_frameNameQueue.front());
        if (status != aditof::Status::OK)
            return; // status;
    }

    m_threadRunning = false;
    return; // status;
}

Status FrameHandlerImpl::readNextFrame(aditof::Frame &frame,
                                       std::string fullFileName) {
    Status status = Status::OK;
    if (m_fullInputFileName.empty() && fullFileName.empty()) {
        LOG(ERROR) << "No input file provided!";
        return Status::GENERIC_ERROR;
    }

    if (fullFileName != m_fullInputFileName) {
        m_fullInputFileName = fullFileName;
        m_pos = 0;
    }

    m_file =
        std::fstream(m_fullOutputFileName, std::ios::in | std::ios::binary);

    if (!m_file) {
        LOG(ERROR) << "Failed open file!";
        return Status::GENERIC_ERROR;
    }

    m_file.seekg(m_pos, std::ios::beg);
    if (m_file.eof()) {
        LOG(WARNING) << "End of file reached! No more frames left to read.";
        m_file.close();
        return Status::UNAVAILABLE;
    }

    m_file.read(reinterpret_cast<char *>(&m_metadataStruct), METADATA_SIZE);

    m_frDetails.width = m_metadataStruct.width;
    m_frDetails.height = m_metadataStruct.height;
    m_frDetails.cameraMode = std::to_string(m_metadataStruct.imagerMode);
    m_frDetails.totalCaptures = 1;

    m_info = ModeInfo::getInstance()->getModeInfo(m_metadataStruct.imagerMode);
    if (!m_info.mode_name.empty()) {
        m_frDetails.type = m_info.mode_name;
    }

    FrameDataDetails frDataDetails;
    frDataDetails.type = "metadata";
    frDataDetails.width = METADATA_SIZE;
    frDataDetails.height = 1;
    m_frDetails.dataDetails.emplace_back(frDataDetails);

    frDataDetails.type = "depth";
    frDataDetails.width = m_metadataStruct.width;
    frDataDetails.height = m_metadataStruct.height;
    m_frDetails.dataDetails.emplace_back(frDataDetails);

    frDataDetails.type = "ab";
    frDataDetails.width = m_metadataStruct.width;
    frDataDetails.height = m_metadataStruct.height;
    m_frDetails.dataDetails.emplace_back(frDataDetails);

    frDataDetails.type = "conf";
    frDataDetails.width = m_metadataStruct.width;
    frDataDetails.height = m_metadataStruct.height;
    m_frDetails.dataDetails.emplace_back(frDataDetails);

    status = frame.setDetails(m_frDetails);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to set frame details.";
        return status;
    }

    m_frDetails.dataDetails.clear();

    //Read frames from the file in followind order: metadata depth ab conf
    uint16_t *metaData;
    uint16_t *depthData;
    uint16_t *abData;
    uint16_t *confData;

    frame.getData("metadata", &metaData);
    memcpy(metaData, reinterpret_cast<uint8_t *>(&m_metadataStruct),
           METADATA_SIZE);

    frame.getData("depth", &depthData);
    frame.getData("ab", &abData);
    frame.getData("conf", &confData);

    //at first we assume that we have metadata enabled by default
    //TO DO: implement use-case where we don't have metadata

    if (m_metadataStruct.bitsInDepth)
        m_file.read(reinterpret_cast<char *>(depthData),
                    m_metadataStruct.width * m_metadataStruct.height * 2);
    if (m_metadataStruct.bitsInAb)
        m_file.read(reinterpret_cast<char *>(abData),
                    m_metadataStruct.width * m_metadataStruct.height * 2);
    if (m_metadataStruct.bitsInConfidence)
        m_file.read(reinterpret_cast<char *>(confData),
                    m_metadataStruct.width * m_metadataStruct.height * 2);

    m_pos = m_file.tellg();
    m_file.close();

    return Status::OK;
}

Status FrameHandlerImpl::setCustomFormat(std::string format) {
    return Status::UNAVAILABLE;
}

Status FrameHandlerImpl::storeFramesToSingleFile(bool enable) {
    Status status = Status::OK;
    m_concatFrames = enable;

    return status;
}

Status FrameHandlerImpl::setFrameContent(std::string frameContent) {
    return Status::UNAVAILABLE;
}

Status FrameHandlerImpl::createFile(std::string fileName) {
    if (fileName.empty()) {
        char time_buffer[128];
        time_t rawtime;
        time(&rawtime);
        struct tm timeinfo;
#ifdef _WIN32
        localtime_s(&timeinfo, &rawtime);
#else
        localtime_r(&rawtime, &timeinfo);
#endif
        strftime(time_buffer, sizeof(time_buffer), "%Y_%m_%d_%H_%M_%S",
                 &timeinfo);
        m_fileName = "frame" + std::string(time_buffer) + "_" +
                     std::to_string(m_frameCount) + ".bin";
        m_frameCount++;
    } else {
        m_fileName = fileName;
    }
    m_fullOutputFileName = m_filePath + '/' + m_fileName;
    m_file = std::fstream(m_fullOutputFileName,
                          std::ios::app | std::ios::out | std::ios::binary);

    if (!m_file) {
        LOG(ERROR) << "Failed to create output file!";
        return Status::GENERIC_ERROR;
    }
    m_fileCreated = true;

    return Status::OK;
}
