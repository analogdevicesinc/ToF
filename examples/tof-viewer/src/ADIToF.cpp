/********************************************************************************/
/*                                                                              */
/* Copyright (c) Microsoft Corporation. All rights reserved.					*/
/*  Portions Copyright (c) 2020 Analog Devices Inc.								*/
/* Licensed under the MIT License.												*/
/*																				*/
/********************************************************************************/

// ADIToFTest.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include <aditof/system.h>
#include <aditof/version.h>
#include <algorithm>
#include <cctype>
#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif
#include <iostream>

#include "ADIMainWindow.h"

#if defined(__APPLE__) && defined(__MACH__)
class GOOGLE_GLOG_DLL_DECL glogLogSink : public google::LogSink {
  public:
    glogLogSink(AppLog *log) : applog(log) {}
    ~glogLogSink() = default;
    virtual void send(google::LogSeverity severity, const char *full_filename,
                      const char *base_filename, int line,
                      const struct ::tm *tm_time, const char *message,
                      size_t message_len) {
        if (applog) {
            std::string msg(message, message_len);
            msg += "\n";
            applog->AddLog(msg.c_str(), nullptr);
        }
    };

  private:
    AppLog *applog = nullptr;
};
#endif

void ProcessArgs(int argc, char **argv, ADIViewerArgs &args);

void ProcessArgs(int argc, char **argv, ADIViewerArgs &args) {
    // Skip argv[0], which is the path to the executable
    //

    for (int i = 1; i < argc; i++) {
        // Force to uppercase
        //
        std::string arg = argv[i];
        std::transform(arg.begin(), arg.end(), arg.begin(),
                       [](unsigned char c) {
                           return static_cast<unsigned char>(std::toupper(c));
                       });

        if (arg == std::string("--HIGHDPI")) {
            args.HighDpi = true;
        } else if (arg == std::string("--NORMALDPI")) {
            args.HighDpi = false;
        }
    }
}

int main(int argc, char **argv) {
    FLAGS_logtostderr = 1;

    auto view = std::make_shared<
        adiMainWindow::ADIMainWindow>(); //Create a new instance

#if defined(__APPLE__) && defined(__MACH__)
    //forward glog messages to GUI log windows
    glogLogSink *sink = new glogLogSink(view->getLog());
    google::AddLogSink(sink);
#endif

    LOG(INFO) << "SDK version: " << aditof::getApiVersion()
              << " | branch: " << aditof::getBranchVersion()
              << " | commit: " << aditof::getCommitVersion();

    ADIViewerArgs args;

    ProcessArgs(argc, argv, args);
    if (view->StartImGUI(args)) {
        view->Render();
    }
    return 0;
}
