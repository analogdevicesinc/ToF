/****************************************************************************
* Copyright (c) 2023 - Analog Devices Inc. All Rights Reserved.
* This software is proprietary & confidential to Analog Devices, Inc.
* and its licensors.
* *****************************************************************************
* *****************************************************************************
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

/* Available executable tools */
1. set_fps - Sets the FPS of the ADSD3500 device.
2. get_dealias_and_intrinsic - Gets Dealias and Camera Intrinsic parameters from the imager.
3. stop_stream - Stops streaming in the ADSD3500 device.
4. update_fw - Updates ADSD3500 firmware.
5. nvm_write - Writes NVM image in the ToF camera module.

/* Steps to create and run the executable that sets FPS value in the ADSD3500 device. */

1. In the terminal run the 'make' command to generate the executable.
    $ make <executable_name> (for example, to build 'set_fps' executable, run the command: $ make set_fps)

2. Copy the executable to Amba board.

3. Run the executables in the board as shown below:
    $ ./set_fps <desired_fps_value>
    $ ./get_dealias_and_intrinsic <imager_mode>
    $ ./stop_stream
    $ ./update_fw <ADSD3500_Firmware.bin>
    $ ./nvm_write <NVM_image.bin>



