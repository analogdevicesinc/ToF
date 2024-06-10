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

#include "utils_ini.h"
#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#include <cstring>
#include <unistd.h>
#endif
#include <fstream>
#include <string>

using namespace std;
using namespace aditof;

Status UtilsIni::getKeyValuePairsFromIni(const string &iniFileName,
                                         map<string, string> &iniKeyValPairs) {

    ifstream iniStream(iniFileName);
    if (!iniStream.is_open()) {
        LOG(ERROR) << "Failed to open: " << iniFileName;
        return Status::UNREACHABLE;
    }

    iniKeyValPairs.clear();

    string line;
    while (getline(iniStream, line)) {
        size_t equalPos = line.find('=');
        if (equalPos == string::npos) {
            LOG(WARNING) << "Unexpected format on this line:\n"
                         << line << "\nExpecting 'key=value' format";
            continue;
        }
        string key = line.substr(0, equalPos);
        string value = line.substr(equalPos + 1);
        if (!value.empty()) {
            iniKeyValPairs.emplace(key, value);
        } else {
            LOG(WARNING) << "No value found for parameter: " << key;
        }
    }

    iniStream.close();

    return Status::OK;
}

Status UtilsIni::getKeyValuePairsFromString(
    const std::string &iniStr,
    std::map<std::string, std::string> &iniKeyValPairs) {
    iniKeyValPairs.clear();
    stringstream ss(iniStr);
    string line;
    char delimiter = '\n';
    while (getline(ss, line, delimiter)) {
        size_t equalPos = line.find('=');
        if (equalPos == string::npos) {
            LOG(WARNING) << "Unexpected format on this line:\n"
                         << line << "\nExpecting 'key=value' format";
            continue;
        }
        string key = line.substr(0, equalPos);
        string value = line.substr(equalPos + 1);
        if (!value.empty()) {
            iniKeyValPairs.emplace(key, value);
        } else {
            LOG(WARNING) << "No value found for parameter: " << key;
        }
    }
    return Status::OK;
}
