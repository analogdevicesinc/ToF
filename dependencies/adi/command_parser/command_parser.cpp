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

void CommandParser::parseArguments(int argc, char *argv[]) {
    // Parse the config and stores argument + value
    for (int i = 1; i < argc; i++) {
        int contains_equal = std::string(argv[i]).find("=");
        if (contains_equal != -1) {
            m_command_vector.push_back(
                {std::string(argv[i]).substr(0, contains_equal),
                 std::string(argv[i]).substr(contains_equal + 1)});
        } else if (std::string(argv[i]) == "-h" ||
                   std::string(argv[i]) == "--help") {
            m_command_vector.push_back({argv[i], "help_menu"});
        } else if (i != argc - 1) {
            if (std::string(argv[i + 1]).find("-") == -1 ||
                std::string(argv[i + 1]).find("-") != 0) {
                    m_command_vector.push_back({argv[i], argv[i + 1]});
                    i++;
            } else {
                m_command_vector.push_back({argv[i], ""});
            }
        } else {
            m_command_vector.push_back({argv[i], ""});
        }
    }
}

int CommandParser::checkArgumentExist(
    std::map<std::string, struct Argument> &command_map,
    std::string &arg_error) {
    // Check if arguments used exist
    for (int i = 0; i < m_command_vector.size(); i++) {
        bool is_command = false;
        for (auto ct = command_map.begin(); ct != command_map.end(); ct++) {
            if (m_command_vector[i].first == ct->first ||
                m_command_vector[i].first == ct->second.long_option) {
                is_command = true;
                break;
            }
        }
        if (!is_command) {
            arg_error = m_command_vector[i].first;
            return -1;
        }
    }
    return 0;
}

int CommandParser::helpMenu() {
    for (int i = 0; i < m_command_vector.size(); i++) {
        if (m_command_vector[i].second == "help_menu") {
            if (i != 0 || m_command_vector.size() != 1) {
                return -1;
            }
            return 1;
        }
    }
    return 0;
}

int CommandParser::checkValue(
    std::map<std::string, struct Argument> &command_map,
    std::string &arg_error) {
    // Checks if argument has default and value assigned.
    // If there is value assigned, it will send it
    for (int i = 0; i < m_command_vector.size(); i++) {
        for (auto ct = command_map.begin(); ct != command_map.end(); ct++) {
            if (m_command_vector[i].first == ct->first ||
                m_command_vector[i].first == ct->second.long_option) {
                if (m_command_vector[i].second == "" &&
                    ct->second.value == "") {
                    arg_error = ct->first;
                    return -1;
                } else if (m_command_vector[i].second == "" &&
                           ct->second.value != "") {
                    // Argument doesn't have value assigned but has default
                    break;
                } else {
                    // Argument has value assigned
                    ct->second.value = m_command_vector[i].second;
                    break;
                }
            }
        }
    }
    return 0;
}

int CommandParser::checkMandatoryArguments(
    std::map<std::string, struct Argument> &command_map,
    std::string &arg_error) {
    // Check if mandatory arguments are provided
    for (auto ct = command_map.begin(); ct != command_map.end(); ct++) {
        if (ct->second.is_mandatory == true && ct->second.value == "") {
            arg_error = ct->first;
            return -1;
        }
    }
    return 0;
}

int CommandParser::checkMandatoryPosition(
    std::map<std::string, struct Argument> &command_map,
    std::string &arg_error) {
    // Mandatory arguments location check
    for (auto ct = command_map.begin(); ct != command_map.end(); ct++) {
        if (ct->second.is_mandatory == true) {
            int index;
            if (ct->second.position != "last") {
                index = std::stoi(ct->second.position);
            } else {
                index = m_command_vector.size() - 1;
            }
            if (m_command_vector[index].first != ct->first &&
                m_command_vector[index].first != ct->second.long_option) {
                arg_error = ct->first;
                return -1;
            }
        }
    }
    return 0;
}