/**********************************************************************************/
/* MIT License                                                                    */
/*                                                                                */
/* Copyright (c) 2021 Analog Devices, Inc.                                        */
/*                                                                                */
/* Permission is hereby granted, free of charge, to any person obtaining a copy   */
/* of this software and associated documentation files (the "Software"), to deal  */
/* in the Software without restriction, including without limitation the rights   */
/* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell      */
/* copies of the Software, and to permit persons to whom the Software is          */
/* furnished to do so, subject to the following conditions:                       */
/*                                                                                */
/* The above copyright notice and this permission notice shall be included in all */
/* copies or substantial portions of the Software.                                */
/*                                                                                */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR     */
/* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,       */
/* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE    */
/* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER         */
/* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,  */
/* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE  */
/* SOFTWARE.                                                                      */
/**********************************************************************************/

#include "command_parser.h"
#include <string>
#include <vector>
#include <iostream>
#include <unordered_map>
#include <regex>

/* CommandParser::CommandParser() {
    {{"-h", "--help"}, "null"},
    {{"-f", "--folder"}, "./"},
    {{"-n", "--ncapture"}, "1"},
    {{"-m", "--mode"}, "0"},
    {{"-ext", "--ext_fsync"}, "0"},
    {{"-wt", "--warmup"}, "0"},
    {{"-ip", "--ip"}, "null"},
    {{"-fw", "--firmware"}, "null"},
    {{"-fps", "--setfps"}, "null"},
    {{"-ccb", "--file"}, "null"},
    {{"FILE"}, "null"}
};*/

std::unordered_map<std::string, std::string> CommandParser::getConfiguration() {
    return m_command_map;
}   

int CommandParser::parseArguments(int argc, char *argv[]) {
    for (int i = 1; i < argc; i++) {
        int end = std::string(argv[i]).find("=");
        if (end != -1)
            m_command_map.insert({std::string(argv[i]).substr(0, end),
                                  std::string(argv[i]).substr(end + 1)});
        else if(i != argc-1) {
            m_command_map.insert({argv[i], argv[i + 1]});
            i++;
        } else if (std::string(argv[i]) == "-h" || std::string(argv[i]) == "--help") {
            m_command_map.insert({argv[i], "help_menu"});
        } else {
            m_command_map.insert({"FILE", argv[i]});
        }
    } 
    return 0;
}