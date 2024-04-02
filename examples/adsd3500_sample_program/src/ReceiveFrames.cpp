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
    auto adsd3500 = Adsd3500();

    ret = adsd3500.ReceiveFrames();
    if (ret < 0) {
        printf("Unable to receive frames.\n");
    }
    
    return 0;
}
