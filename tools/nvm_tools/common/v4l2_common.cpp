/****************************************************************************
# Copyright (c) 2024 - Analog Devices Inc. All Rights Reserved.
# This software is proprietary & confidential to Analog Devices, Inc.
# and its licensors.
# *****************************************************************************
# *****************************************************************************/

#include "include/v4l2_common.h"

#include <iostream>
#include <array>
#include <cstdio>
#include <cstring>
#include <errno.h>
#include <malloc.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

using namespace std;

#define IOCTL_TRIES 1
#define CTRL_SIZE 4099

std::string find_media_device_with_entity(const std::string &entity_name)
{
    for (int i = 0; i <= 3; i++)
    {
        std::string media_dev = "/dev/media" + std::to_string(i);
        std::string cmd = "media-ctl -d " + media_dev + " --print-dot 2>/dev/null";

        std::array<char, 256> buffer{};
        std::string dot_output;

        FILE *pipe = popen(cmd.c_str(), "r");
        if (!pipe)
            continue;

        while (fgets(buffer.data(), buffer.size(), pipe))
            dot_output += buffer.data();

        pclose(pipe);

        if (dot_output.empty())
            continue;

        if (dot_output.find(entity_name) != std::string::npos)
            return media_dev;
    }

    return "";
}

std::string find_subdev_in_media(const std::string &media_dev,
                                 const std::string &entity_name)
{
    std::string cmd = "media-ctl -d " + media_dev + " --print-dot 2>/dev/null";

    std::array<char, 256> buffer{};
    std::string dot;

    FILE *pipe = popen(cmd.c_str(), "r");
    if (!pipe)
        return "";

    while (fgets(buffer.data(), buffer.size(), pipe))
        dot += buffer.data();

    pclose(pipe);

    if (dot.empty())
        return "";

    size_t pos = dot.find(entity_name);
    if (pos == std::string::npos)
        return "";

    size_t dev_pos = dot.find("/dev/v4l-subdev", pos);
    if (dev_pos == std::string::npos)
        return "";

    size_t end = dot.find_first_of(" \"\n", dev_pos);
    return dot.substr(dev_pos, end - dev_pos);
}

bool findDevicePathsAtVideo(const std::string &video, std::string &subdev_path,
                            std::string &device_name) {

    char *buf;
    int size = 0;
    size_t pos;

    /* Run media-ctl to get the video processing pipes */
    char cmd[64];
    sprintf(cmd, "media-ctl -d %s --print-dot", video.c_str());
    FILE *fp = popen(cmd, "r");
    if (!fp) {
        std::cout << "Error running media-ctl";
        return false;
    }

    /* Read the media-ctl output stream */
    buf = (char *)malloc(128 * 1024);
    while (!feof(fp)) {
        auto sz = fread(&buf[size], 1, 1, fp);
        size++;
    }
    pclose(fp);
    buf[size] = '\0';

    /* Search command media-ctl for device/subdevice name */
    string str(buf);
    free(buf);

    if (str.find("adsd3500") != string::npos) {
        device_name = "adsd3500";
        pos = str.find("adsd3500");
        subdev_path = str.substr(pos + strlen("adsd3500") + 9,
                                 strlen("/dev/v4l-subdevX"));
    } else {
        return false;
    }
    return true;
}

int xioctl(int fd, int request, void *arg) {
    int r;
    int tries = IOCTL_TRIES;
    do {
        r = ioctl(fd, request, arg);
    } while (--tries > 0 && r == -1 && EINTR == errno);

    return r;
}

bool v4l2_ctrl_set(int fd, uint32_t id, uint8_t *val) {
    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;

    extCtrl.size = CTRL_SIZE * sizeof(char);
    extCtrl.p_u8 = val;
    extCtrl.id = id;
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;
    if (xioctl(fd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        std::cout << "Failed to set ctrl with id " << id << std::endl;
        return false;
    }

    return true;
}

bool v4l2_ctrl_get(int fd, uint32_t id, uint8_t *val) {
    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;

    extCtrl.size = CTRL_SIZE * sizeof(char);
    extCtrl.p_u8 = val;
    extCtrl.id = id;
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;
    if (xioctl(fd, VIDIOC_G_EXT_CTRLS, &extCtrls) == -1) {
        std::cout << "Failed to get ctrl with id " << id << std::endl;
        return false;
    }

    return true;
}
