/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
#include <string>
#include <vector>

#ifndef ADI_OPEN_FILE
#define ADI_OPEN_FILE

extern std::string customFilter;
extern std::vector<std::string> customFilters;

/**
* @brief Returns an empty string if cancelled.
*                      Opens a dialog box to fetch a file with a custom file extension
* @param filter        Is assigned a customized filter per camera requirements
* @param owner         Platform specific, typically NULL
* @return              Selected file name with its extension
*/
std::string openADIFileName(const char* filter = customFilter.c_str(), void *owner = NULL);

/**
* @brief Opens a dialog box to save a file
* @param hwndOwner     Platform specific, current handle or NULL
* @param filename      Saved name from user
* @param FilterIndex   Index of chosen filter
* @return              Saved name if successful, empty string otherwise
*/
std::string getADIFileName(void *hwndOwner, char* filename,  int & FilterIndex);

/**
* @brief Finds a set of files with specified file extension
* @param filePath       User selected file path
* @param extension      File extension to be found
* @param returnFileName Set of files that match the chosen extension
* @param returnFullPath If true: returnFileName is returned with both
                        path and file name, otherwise just the file name
*/
void getFilesList(std::string filePath, std::string extension, std::vector<std::string>& returnFileName, bool returnFullPath);

#endif //ADI_OPEN_FILE
