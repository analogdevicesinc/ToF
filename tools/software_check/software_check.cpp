/*
 * MIT License
 *
 * Copyright (c) 2023, Analog Devices, Inc.
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
#include "c_json/cJSON.h"
#include <aditof/camera.h>
#include <aditof/system.h>
#include <aditof/version.h>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

const char *localJson = "sw_versions.info";
const char *remoteJsonURL =
    "https://swdownloads.analog.com/cse/aditof/aware3d/sw_update/remote.json";
const char *localTmpFile = "remote.json";
std::string ip = "192.168.56.1";
const char *VERSION = "v1.0.1";

int32_t getLocalJSON(std::string &fwversion, std::string &fwhash,
                     std::string &sdcard, std::string &package_sdk,
                     std::string &branch, std::string &commit) {
    // Open the JSON file
    std::ifstream file(localJson);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << localJson << std::endl;
        return -1;
    }

    // Read the JSON contents into a string buffer
    std::string buffer((std::istreambuf_iterator<char>(file)),
                       std::istreambuf_iterator<char>());

    // Parse the JSON contents
    cJSON *data = cJSON_Parse(buffer.c_str());
    if (!data) {
        std::cerr << "Error parsing JSON!" << std::endl;
        return -2;
    }

    cJSON *output;
    output = cJSON_GetObjectItem(data, "firmware");
    fwversion = (output != nullptr) ? output->valuestring : "";

    output = cJSON_GetObjectItem(data, "firmwarehash");
    fwhash = (output != nullptr) ? output->valuestring : "";

    output = cJSON_GetObjectItem(data, "sdcard");
    sdcard = (output != nullptr) ? output->valuestring : "";

    output = cJSON_GetObjectItem(data, "package-sdk");
    package_sdk = (output != nullptr) ? output->valuestring : "";

    output = cJSON_GetObjectItem(data, "branch");
    branch = (output != nullptr) ? output->valuestring : "";

    output = cJSON_GetObjectItem(data, "commit");
    commit = (output != nullptr) ? output->valuestring : "";

    // Free the memory
    cJSON_Delete(data);
    return 0;
}

int32_t getinstalledVersion(std::string &fwVersion, std::string &fwHash,
                            std::string &sdcard, std::string &sdkpackage,
                            std::string &sdkbranch, std::string &sdkcommit) {

    sdkpackage = aditof::getApiVersion();
    sdkbranch = aditof::getBranchVersion();
    sdkcommit = aditof::getCommitVersion();

    aditof::System system;
    std::vector<std::shared_ptr<aditof::Camera>> cameras;

    if (!ip.empty()) {
        ip = "ip:" + ip;
    }
    system.getCameraList(cameras, ip);
    if (cameras.empty()) {
        std::cout << "No cameras found" << std::endl;
        return -1;
    }

    auto camera = cameras.front();

    aditof::Status status = aditof::Status::OK;
    status = camera->initialize();
    if (status != aditof::Status::OK) {
        std::cout << "Could not initialize camera!" << std::endl;
        return -2;
    }

    aditof::CameraDetails cameraDetails;
    camera->getDetails(cameraDetails);

    camera->adsd3500GetFirmwareVersion(fwVersion, fwHash);

    sdcard = cameraDetails.sdCardImageVersion;

    return 0;
}

int32_t getRemoteJSON(std::string &packageversion, std::string &packageurl) {

    std::string url = remoteJsonURL;
    std::string filename = localTmpFile;

    std::filesystem::remove(filename);

    // Create an HTTP request using cURL and save the response to a file
    std::system(
        ("curl -H \"Cache-Control: no-cache\" -s -o " + filename + " " + url)
            .c_str());

    // Read the file into memory
    std::ifstream file(filename);
    std::stringstream buffer;
    buffer << file.rdbuf();

    // Parse the JSON contents
    cJSON *data = cJSON_Parse(buffer.str().c_str());
    if (!data) {
        std::cerr << "Error parsing JSON: " << buffer.str().c_str()
                  << std::endl;
        std::filesystem::remove(filename);
        return -1;
    }

    // Extract the values from the JSON object
    cJSON *output;
    output = cJSON_GetObjectItem(data, "packageversion");
    packageversion = (output != nullptr) ? output->valuestring : "";
    output = cJSON_GetObjectItem(data, "packageurl");
    packageurl = (output != nullptr) ? output->valuestring : "";
    // Free the memory and cleanup
    cJSON_Delete(data);
    file.close();
    std::filesystem::remove(filename);

    return 0;
}

std::string compare(const std::string s1, const std::string s2) {
    if (s1 == s2) {
        return std::string(" == ");
    } else {
        return std::string(" != ");
    }
}

int main(int argc, char *argv[]) {
    int32_t ret = 0;

    std::cout << "SofwareCheck " << VERSION << std::endl << std::endl;

    if (argc != 2) {
        std::cerr << "Usage:" << std::endl;
        std::cerr << argv[0] << " "
                  << "<ip address>" << std::endl;
        std::cerr << "Where the 'ip address' is the address of the eval kit, "
                     "normally 192.168.56.1"
                  << std::endl;
        std::cerr << std::endl;
        std::cerr << "For example:" << std::endl;
        std::cerr << argv[0] << " "
                  << "192.168.56.1" << std::endl;
        std::cerr << std::endl;
        std::cerr << "Using the default IP address of " << ip << std::endl;
    }

    if (argc == 2) {
        ip = argv[1];
        std::cout
            << "No IP address specififed, using network access with IP address "
            << ip << "." << std::endl
            << std::endl;
    }

    // Version as stored on the server.
    std::string remotePackageversion;
    std::string remotePackageurl;
    int32_t remotestatus;
    remotestatus = getRemoteJSON(remotePackageversion, remotePackageurl);

    if (remotestatus < 0) {
        std::cerr << "Warning: Unable to get details on new version "
                     "availability. Try again later. Continuing.";
    }

    // Version information as distributed in the package.
    std::string distributedFwversion;
    std::string distributedFwhash;
    std::string distributedSdcard;
    std::string distributedPackageversion;
    std::string distributedBranch;
    std::string distributedCommit;
    ret = getLocalJSON(distributedFwversion, distributedFwhash,
                       distributedSdcard, distributedPackageversion,
                       distributedBranch, distributedCommit);

    if (ret < 0) {
        std::cerr
            << "Error: Unable to process local version information. Exiting.";
        return ret;
    }

    // Version information from the device.
    std::string installedFwversion;
    std::string installedFwhash;
    std::string installedSdcard;
    std::string installedSdkPackageversion;
    std::string installedSdkBranch;
    std::string installedSdkCommit;
    std::cerr << "*******************************: Getting Installed Version "
                 "Infomation: start"
              << std::endl;
    std::cerr
        << "CTRL+C if call does not return. Start by updating the SD card."
        << std::endl;
    ret = getinstalledVersion(installedFwversion, installedFwhash,
                              installedSdcard, installedSdkPackageversion,
                              installedSdkBranch, installedSdkCommit);

    std::cerr << "*******************************: Getting Installed Version "
                 "Infomation: stop"
              << std::endl;

    if (ret < 0) {
        std::cerr
            << "Error: Unable to get installed version information. Please "
               "start by ensuring the SD card is updated. Exiting.";
        return ret;
    }

    if (distributedFwversion != installedFwversion ||
        distributedFwhash != installedFwhash) {
        std::cerr << std::endl;
        std::cerr << "** Error: Firmware version you have installed is "
                     "incorrect: distributed != installed"
                  << std::endl;
        std::cerr << "    Firmware version: " << distributedFwversion
                  << compare(distributedFwversion, installedFwversion)
                  << installedFwversion << std::endl;
        std::cerr << "    Firmware git commit hash: " << distributedFwhash
                  << compare(distributedFwhash, installedFwhash)
                  << installedFwhash << std::endl;
    }
    if (distributedSdcard != installedSdcard) {
        std::cerr << std::endl;
        std::cerr << "** Error: SD Card version you have used is incorrect: "
                     "distributed != installed"
                  << std::endl;
        std::cerr << "    SD Card version: " << distributedSdcard
                  << compare(distributedSdcard, installedSdcard)
                  << installedSdcard << std::endl;
    }
    if (distributedPackageversion != installedSdkPackageversion ||
        distributedBranch != installedSdkBranch ||
        distributedCommit != installedSdkCommit) {
        std::cerr << std::endl;
        std::cerr << "** Error: Host eval package installed does not match "
                     "expected version: distributed != installed"
                  << std::endl;
        std::cerr << "    Package version: " << distributedPackageversion
                  << compare(distributedPackageversion,
                             installedSdkPackageversion)
                  << installedSdkPackageversion << std::endl;
        std::cerr << "    Package branch: " << distributedBranch
                  << compare(distributedBranch, installedSdkBranch)
                  << installedSdkBranch << std::endl;
        std::cerr << "    Pacakge commit hash: " << distributedCommit
                  << compare(distributedCommit, installedSdkCommit)
                  << installedSdkCommit << std::endl;
    }

    if (remotestatus >= 0 &&
        remotePackageversion != distributedPackageversion) {
        std::cerr << std::endl;
        std::cerr << ">> New update is avialable: " << remotePackageversion
                  << std::endl;
        std::cerr << ">> Please see, " << remotePackageurl << std::endl;
    }

    return 0;
}
