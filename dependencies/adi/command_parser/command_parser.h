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

#include <map>
#include <string>
#include <vector>

struct Argument {
    std::string long_option;
    bool is_mandatory;
    std::string position;
    std::string value;
    bool has_value;
};

class CommandParser {
  public:
    CommandParser() = default;
    ~CommandParser() = default;

  public:
    void parseArguments(int argc, char *argv[],
                        std::map<std::string, struct Argument> command_map);
    int checkArgumentExist(std::map<std::string, struct Argument> &command_map,
                           std::string &arg_error);
    int checkValue(std::map<std::string, struct Argument> &command_map,
                   std::string &arg_error);
    int
    checkMandatoryArguments(std::map<std::string, struct Argument> &command_map,
                            std::string &arg_error);
    int
    checkMandatoryPosition(std::map<std::string, struct Argument> &command_map,
                           std::string &arg_error);
    int helpMenu();

  private:
    std::vector<std::pair<std::string, std::string>> m_command_vector;
};