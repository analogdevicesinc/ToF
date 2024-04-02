/*****************************************************************************
* Copyright (c) 2023 - Analog Devices Inc. All Rights Reserved.
* This software is proprietary & confidential to Analog Devices, Inc.
* and its licensors.
*******************************************************************************
*******************************************************************************
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.*/

#include <iostream>
#include "../include/adsd3500_util.h"

int main(int argc, char *argv[]) {

    int ret;

    if (argc != 2) {
        printf("set_image_mode usage: \n");
        printf("./set_image_mode <mode_number>\n");
        return 0;
    }

    int modeNumber = atoi(argv[1]);

    if (modeNumber < 0 || modeNumber > 4) {
        printf("Invalid mode number given. Valid Imaging modes are 0, 1, 2, 3 and 4.\n");
        return 0;
    }

    auto adsd3500 = Adsd3500();

    ret = adsd3500.SetImageMode(modeNumber);
    if (ret < 0) {
        printf("Unable to set the Image Mode in Adsd3500.\n");
    }

    uint8_t set_image_mode[2] = {0x00, 0x00};

    ret = adsd3500.GetImageMode(set_image_mode);
    if (ret < 0) {
        printf("Unable to get the Image Mode from Adsd3500.\n");
    }

    PrintByteArray(set_image_mode, ARRAY_SIZE(set_image_mode));

    return 0;
}
