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
#include "aditofdemorecorder.h"

#include <functional>
#include <string.h>

AditofDemoRecorder::AditofDemoRecorder()
    : m_frameDataDetails{}, m_recordTreadStop(true), m_playbackThreadStop(true),
      m_shouldReadNewFrame(true), m_playBackEofReached(false),
      m_numberOfFrames(0) {}

AditofDemoRecorder::~AditofDemoRecorder() {
    if (m_recordFile.is_open()) {
        stopRecording();
    }
    if (m_playbackFile.is_open()) {
        stopPlayback();
    }
}

void AditofDemoRecorder::startRecording(
    const std::string &fileName,
    const std::vector<aditof::FrameDataDetails> &frameDataDetails,
    unsigned int fps) {
    m_recordFile.open(fileName, std::ios::binary);
    m_recordFile.write(reinterpret_cast<const char *>(&fps),
                       sizeof(unsigned int));
    unsigned int numDetails = frameDataDetails.size();
    m_recordFile.write(reinterpret_cast<const char *>(&numDetails),
                       sizeof(unsigned int));
    for (const auto &i : frameDataDetails) {
        m_recordFile.write(i.type.c_str(), i.type.size());
        m_recordFile.write("\n", 1);
        m_recordFile.write(reinterpret_cast<const char *>(&i.height),
                           sizeof(unsigned int));
        m_recordFile.write(reinterpret_cast<const char *>(&i.width),
                           sizeof(unsigned int));
    }

    m_frameDataDetails = frameDataDetails;

    m_recordTreadStop = false;
    m_recordThread =
        std::thread(std::bind(&AditofDemoRecorder::recordThread, this));
}

void AditofDemoRecorder::stopRecording() {
    m_recordTreadStop = true;
    if (m_recordThread.joinable()) {
        m_recordThread.join();
    }
    m_recordFile.close();
}

int AditofDemoRecorder::startPlayback(const std::string &fileName, int &fps) {
    m_playbackFile.open(fileName, std::ios::binary);

    m_playbackFile.seekg(0, std::ios_base::end);
    int fileSize = m_playbackFile.tellg();
    m_playbackFile.seekg(0, std::ios_base::beg);

    m_playbackFile.read(reinterpret_cast<char *>(&fps), sizeof(int));
    unsigned int numDetails;
    m_playbackFile.read(reinterpret_cast<char *>(&numDetails),
                        sizeof(unsigned int));

    m_frameDataDetails.clear();
    
    for (unsigned int i = 0; i < numDetails; ++i) {
        aditof::FrameDataDetails fDetails;

        std::getline(m_playbackFile, fDetails.type);
        m_playbackFile.read(reinterpret_cast<char *>(&fDetails.height),
                            sizeof(unsigned int));
        m_playbackFile.read(reinterpret_cast<char *>(&fDetails.width),
                            sizeof(unsigned int));
        m_frameDataDetails.emplace_back(fDetails);
    }

    int sizeOfFrame = 0;
    int sizeOfHeader = sizeof(fps);
    for (const auto &i : m_frameDataDetails) {
        sizeOfHeader += i.type.size() + 1; // 1 is for the new line character
        sizeOfHeader += sizeof(i.width) + sizeof(i.height);
        sizeOfFrame += sizeof(uint16_t) * i.height * i.width;
    }

    m_numberOfFrames = (fileSize - sizeOfHeader) / sizeOfFrame;

    m_playbackThreadStop = false;
    m_playBackEofReached = false;
    m_playbackThread =
        std::thread(std::bind(&AditofDemoRecorder::playbackThread, this));

    return m_numberOfFrames;
}

void AditofDemoRecorder::stopPlayback() {
    m_playbackThreadStop = true;
    std::unique_lock<std::mutex> lock(m_playbackMutex);
    m_shouldReadNewFrame = true;
    lock.unlock();
    m_playbackCv.notify_one();
    if (m_playbackThread.joinable()) {
        m_playbackThread.join();
    }
    m_playbackFile.close();
}

void AditofDemoRecorder::recordNewFrame(std::shared_ptr<aditof::Frame> frame) {
    m_recordQueue.enqueue(frame);
}

std::shared_ptr<aditof::Frame> AditofDemoRecorder::readNewFrame() {
    return m_playbackQueue.dequeue();
}

void AditofDemoRecorder::requestFrame() {
    std::unique_lock<std::mutex> lock(m_playbackMutex);
    m_shouldReadNewFrame = true;
    lock.unlock();
    m_playbackCv.notify_one();
}

bool AditofDemoRecorder::isRecordingEnabled() const {
    return !m_recordTreadStop;
}

bool AditofDemoRecorder::isPlaybackEnabled() const {
    return !m_playbackThreadStop;
}

bool AditofDemoRecorder::isPlaybackFinished() const {
    return m_playBackEofReached;
}

int AditofDemoRecorder::getNumberOfFrames() const { return m_numberOfFrames; }

void AditofDemoRecorder::recordThread() {
    while (!m_recordTreadStop) {

        if (!m_recordFile.is_open()) {
            break;
        }

        if (m_recordQueue.empty()) {
            continue;
        }

        auto frame = m_recordQueue.dequeue();

        for (const auto &i : m_frameDataDetails) {
            uint16_t *data;
            frame->getData(i.type, &data);
            unsigned int size = static_cast<unsigned int>(sizeof(uint16_t) *
                                                          i.width * i.height);
            m_recordFile.write(reinterpret_cast<const char *>(data), size);
        }
    }
}

void AditofDemoRecorder::playbackThread() {
    while (!m_playbackThreadStop) {

        if (!m_playbackFile.is_open()) {
            break;
        }

        std::unique_lock<std::mutex> lock(m_playbackMutex);
        m_playbackCv.wait(lock, [&]() { return m_shouldReadNewFrame; });
        m_shouldReadNewFrame = false;

        if (m_playbackThreadStop) {
            break;
        }

        std::shared_ptr<aditof::Frame> frame =
            std::make_shared<aditof::Frame>();
        ;

        aditof::FrameDetails frameDetails;
        frameDetails.dataDetails = m_frameDataDetails;
        frame->setDetails(frameDetails);

        for (const auto &i : m_frameDataDetails) {
            uint16_t *frameDataLocation;
            frame->getData(i.type, &frameDataLocation);

            if (m_playbackFile.eof()) {
                memset(frameDataLocation, 0,
                       sizeof(uint16_t) * i.width * i.height);
                m_playBackEofReached = true;
            } else {
                int size =
                    static_cast<int>(sizeof(uint16_t) * i.width * i.height);
                m_playbackFile.read(reinterpret_cast<char *>(frameDataLocation),
                                    size);
            }
        }

        m_playbackQueue.enqueue(frame);
    }
}
