/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/

#include "device_parameters.h"

aditof::Status DeviceParameters::createIniParams(
    std::vector<iniFileStruct> &iniFileStructList) {

    using namespace std;

    map<string, string> adsd3030_0 = {{"abThreshMin", "3.0"},
                                      {"confThresh", "25.0"},
                                      {"radialThreshMin", "100.0"},
                                      {"radialThreshMax", "10000.0"},
                                      {"jblfApplyFlag", "1"},
                                      {"jblfWindowSize", "7"},
                                      {"jblfGaussianSigma", "10.0"},
                                      {"jblfExponentialTerm", "5.0"},
                                      {"jblfMaxEdge", "12.0"},
                                      {"jblfABThreshold", "10.0"},
                                      {"headerSize", "128"},
                                      {"inputFormat", "raw8"},
                                      {"depthComputeIspEnable", "1"},
                                      {"partialDepthEnable", "0"},
                                      {"interleavingEnable", "1"},
                                      {"bitsInPhaseOrDepth", "16"},
                                      {"bitsInConf", "8"},
                                      {"bitsInAB", "16"},
                                      {"phaseInvalid", "0"},
                                      {"xyzEnable", "1"},
                                      {"fps", "16"}};

    iniFileStruct iniFileS0;
    iniFileS0.imagerName = "adsd3030";
    iniFileS0.modeName = "0";
    iniFileS0.iniKeyValPairs = adsd3030_0;
    iniFileStructList.emplace_back(iniFileS0);

    iniFileStruct iniFileS1;
    iniFileS1.imagerName = "adsd3030";
    iniFileS1.modeName = "1";
    iniFileS1.iniKeyValPairs = adsd3030_0;
    iniFileStructList.emplace_back(iniFileS1);

    iniFileStruct iniFileS2;
    iniFileS2.imagerName = "adsd3030";
    iniFileS2.modeName = "2";
    iniFileS2.iniKeyValPairs = adsd3030_0;
    iniFileS2.iniKeyValPairs["fps"] = "40";
    iniFileStructList.emplace_back(iniFileS2);

    iniFileStruct iniFileS3;
    iniFileS3.imagerName = "adsd3030";
    iniFileS3.modeName = "3";
    iniFileS3.iniKeyValPairs = adsd3030_0;
    iniFileS2.iniKeyValPairs["fps"] = "40";
    iniFileStructList.emplace_back(iniFileS3);

    iniFileStruct iniFileS4;
    iniFileS4.imagerName = "adsd3030";
    iniFileS4.modeName = "4";
    iniFileS4.iniKeyValPairs = adsd3030_0;
    iniFileS4.iniKeyValPairs["headerSize"] = "0";
    iniFileS4.iniKeyValPairs["inputFormat"] = "mipiRaw12_8";
    iniFileS4.iniKeyValPairs["depthComputeIspEnable"] = "0";
    iniFileS4.iniKeyValPairs["partialDepthEnable"] = "0";
    iniFileS4.iniKeyValPairs["interleavingEnable"] = "0";
    iniFileS4.iniKeyValPairs["bitsInPhaseOrDepth"] = "0";
    iniFileS4.iniKeyValPairs["bitsInConf"] = "0";
    iniFileS4.iniKeyValPairs["bitsInAB"] = "0";
    iniFileS4.iniKeyValPairs["phaseInvalid"] = "0";
    iniFileS4.iniKeyValPairs["xyzEnable"] = "0";
    iniFileS4.iniKeyValPairs["fps"] = "10";
    iniFileStructList.emplace_back(iniFileS4);

    iniFileStruct iniFileS5;
    iniFileS5.imagerName = "adsd3030";
    iniFileS5.modeName = "5";
    iniFileS5.iniKeyValPairs = adsd3030_0;
    iniFileS5.iniKeyValPairs["abThreshMin"] = "1.0";
    iniFileS5.iniKeyValPairs["jblfWindowSize"] = "5";
    iniFileS5.iniKeyValPairs["fps"] = "40";
    iniFileStructList.emplace_back(iniFileS5);

    iniFileStruct iniFileS6;
    iniFileS6.imagerName = "adsd3030";
    iniFileS6.modeName = "6";
    iniFileS6.iniKeyValPairs = adsd3030_0;
    iniFileS6.iniKeyValPairs["abThreshMin"] = "1.0";
    iniFileS6.iniKeyValPairs["jblfWindowSize"] = "5";
    iniFileS6.iniKeyValPairs["fps"] = "40";
    iniFileStructList.emplace_back(iniFileS6);

    map<string, string> adsd3100_0 = {{"abThreshMin", "3.0"},
                                      {"confThresh", "25.0"},
                                      {"radialThreshMin", "30.0"},
                                      {"radialThreshMax", "4200.0"},
                                      {"jblfApplyFlag", "1"},
                                      {"jblfWindowSize", "7"},
                                      {"jblfGaussianSigma", "10.0"},
                                      {"jblfExponentialTerm", "5.0"},
                                      {"jblfMaxEdge", "12.0"},
                                      {"jblfABThreshold", "10.0"},
                                      {"headerSize", "128"},
                                      {"inputFormat", "mipiRaw12_8"},
                                      {"depthComputeIspEnable", "1"},
                                      {"partialDepthEnable", "1"},
                                      {"interleavingEnable", "0"},
                                      {"bitsInPhaseOrDepth", "12"},
                                      {"bitsInConf", "0"},
                                      {"bitsInAB", "16"},
                                      {"phaseInvalid", "0"},
                                      {"xyzEnable", "1"},
                                      {"fps", "10"}};

    iniFileStruct iniFileSC0;
    iniFileSC0.imagerName = "adsd3100";
    iniFileSC0.modeName = "0";
    iniFileSC0.iniKeyValPairs = adsd3100_0;
    iniFileStructList.emplace_back(iniFileSC0);

    iniFileStruct iniFileSC1;
    iniFileSC1.imagerName = "adsd3100";
    iniFileSC1.modeName = "1";
    iniFileSC1.iniKeyValPairs = adsd3100_0;
    iniFileSC1.iniKeyValPairs["radialThreshMin"] = "50.0";
    iniFileSC1.iniKeyValPairs["radialThreshMax"] = "15000.0";
    iniFileStructList.emplace_back(iniFileSC1);

    iniFileStruct iniFileSC2;
    iniFileSC2.imagerName = "adsd3100";
    iniFileSC2.modeName = "2";
    iniFileSC2.iniKeyValPairs = adsd3100_0;
    iniFileSC2.iniKeyValPairs["inputFormat"] = "raw8";
    iniFileSC2.iniKeyValPairs["partialDepthEnable"] = "0";
    iniFileSC2.iniKeyValPairs["interleavingEnable"] = "1";
    iniFileSC2.iniKeyValPairs["bitsInPhaseOrDepth"] = "16";
    iniFileSC2.iniKeyValPairs["bitsInConf"] = "8";
    iniFileSC2.iniKeyValPairs["fps"] = "16";
    iniFileStructList.emplace_back(iniFileSC2);

    iniFileStruct iniFileSC3;
    iniFileSC3.imagerName = "adsd3100";
    iniFileSC3.modeName = "3";
    iniFileSC3.iniKeyValPairs = adsd3100_0;
    iniFileSC3.iniKeyValPairs["radialThreshMin"] = "50.0";
    iniFileSC3.iniKeyValPairs["radialThreshMax"] = "15000.0";
    iniFileSC3.iniKeyValPairs["inputFormat"] = "raw8";
    iniFileSC3.iniKeyValPairs["partialDepthEnable"] = "0";
    iniFileSC3.iniKeyValPairs["interleavingEnable"] = "1";
    iniFileSC3.iniKeyValPairs["bitsInPhaseOrDepth"] = "16";
    iniFileSC3.iniKeyValPairs["bitsInConf"] = "8";
    iniFileSC3.iniKeyValPairs["fps"] = "16";
    iniFileStructList.emplace_back(iniFileSC3);

    iniFileS4.imagerName = "adsd3100";
    iniFileS4.modeName = "4";
    iniFileStructList.emplace_back(iniFileS4);

    iniFileStruct iniFileSC5;
    iniFileSC5.imagerName = "adsd3100";
    iniFileSC5.modeName = "5";
    iniFileSC5.iniKeyValPairs = adsd3100_0;
    iniFileSC5.iniKeyValPairs["abThreshMin"] = "1.0";
    iniFileSC5.iniKeyValPairs["radialThreshMin"] = "50.0";
    iniFileSC5.iniKeyValPairs["radialThreshMax"] = "15000.0";
    iniFileSC5.iniKeyValPairs["jblfWindowSize"] = "5";
    iniFileSC5.iniKeyValPairs["inputFormat"] = "raw8";
    iniFileSC5.iniKeyValPairs["partialDepthEnable"] = "0";
    iniFileSC5.iniKeyValPairs["interleavingEnable"] = "1";
    iniFileSC5.iniKeyValPairs["bitsInPhaseOrDepth"] = "16";
    iniFileSC5.iniKeyValPairs["bitsInConf"] = "8";
    iniFileSC5.iniKeyValPairs["fps"] = "16";
    iniFileStructList.emplace_back(iniFileSC5);

    iniFileStruct iniFileSC6;
    iniFileSC6.imagerName = "adsd3100";
    iniFileSC6.modeName = "6";
    iniFileSC6.iniKeyValPairs = adsd3100_0;
    iniFileSC5.iniKeyValPairs["abThreshMin"] = "1.0";
    iniFileSC5.iniKeyValPairs["jblfWindowSize"] = "5";
    iniFileSC5.iniKeyValPairs["inputFormat"] = "raw8";
    iniFileSC5.iniKeyValPairs["partialDepthEnable"] = "0";
    iniFileSC5.iniKeyValPairs["interleavingEnable"] = "1";
    iniFileSC5.iniKeyValPairs["bitsInPhaseOrDepth"] = "16";
    iniFileSC5.iniKeyValPairs["bitsInConf"] = "8";
    iniFileSC5.iniKeyValPairs["fps"] = "16";
    iniFileStructList.emplace_back(iniFileSC6);

    return aditof::Status::OK;
}