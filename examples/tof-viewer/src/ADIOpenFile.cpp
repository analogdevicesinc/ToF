/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
#include <iostream>
#include "ADIOpenFile.h"

std::string customFilter("All Files (*.*)\0*.*\0\0", 21);
std::vector<std::string> customFilters;

#ifdef _WIN32
#include <codecvt>
#include <windows.h>
#include <Commdlg.h>

using namespace std;

/**
* @brief		Returns an empty string if cancelled.
*				Opens a dialog box to fetch a file with 
*				a custom file extension
* @param filter	Is assigned a customized filter per
*				camera requirements
* @param owner	NULL
* @return		Selected file name with its extension
*/
string openADIFileName(const char* filter, void *owner)
{
	OPENFILENAME file;
	char fileName[MAX_PATH] = "";
	ZeroMemory(&file, sizeof(file));
	file.lStructSize = sizeof(OPENFILENAME);
	file.hwndOwner = reinterpret_cast<HWND>(owner);
	file.lpstrFilter = filter;
	file.lpstrFile = fileName;
	file.nMaxFile = MAX_PATH;
	file.Flags = OFN_EXPLORER | OFN_FILEMUSTEXIST | OFN_HIDEREADONLY;
	file.lpstrDefExt = '\0';
	string fileNameStr;
	if (GetOpenFileName(&file)) {
		fileNameStr = std::string(fileName);
	}
	return fileNameStr;
} 

/**
* @brief Opens a dialog box to save a file
* @param hwndOwner		urrent handle
* @param filename		Saved name from user
* @param FilterIndex	Index of chosen filter
* @return				Saved name if successful,
*						empty string otherwise
*/
string getADIFileName(void *hwndOwner, char* filename,  int & FilterIndex)
{
	OPENFILENAME ofn = { 0 };
	ofn.lStructSize = sizeof(ofn);
	ofn.Flags = OFN_EXPLORER | OFN_PATHMUSTEXIST;
	ofn.hInstance = GetModuleHandle(0);
	ofn.hwndOwner = reinterpret_cast<HWND>(hwndOwner);
	ofn.nMaxFile = MAX_PATH;
	ofn.lpstrFile = filename;
	ofn.nFilterIndex = 1;
	ofn.lpstrFilter = customFilter.c_str();
	ofn.lpstrDefExt = '\0';
	if (GetSaveFileName(&ofn))
	{
		FilterIndex = (DWORD)ofn.nFilterIndex;
		return filename;
	}
	return "";//Something wen't wrong, so it did not save anything
}

/**
* @brief					Finds a set of files with specified file extension
* @param filePath			User selected file path
* @param extension			File extension to be found
* @param returnFullPath     If true: returnFileName is returned with both
							path and file name, otherwise just the file name
* @return returnFileName	Set of files that match the chosen extension
*/
void getFilesList(string filePath, string extension, vector<string>& returnFileName, bool returnFullPath)
{
	WIN32_FIND_DATA fileInfo;
	HANDLE hFind;
	//if "filePath" does not end with "\\" then we need to add it
	if (filePath.back() != ('\\'))
	{
		filePath += "\\";
	}
	string  fullPath = filePath + extension;	
	hFind = FindFirstFile(fullPath.c_str(), &fileInfo);
	if (hFind != INVALID_HANDLE_VALUE) 
	{
		if (returnFullPath)
		{
			returnFileName.push_back(filePath + fileInfo.cFileName);
		}
		else
		{
			returnFileName.push_back(fileInfo.cFileName);
		}
		while (FindNextFile(hFind, &fileInfo) != 0) 
		{
			if (returnFullPath)
			{
				returnFileName.push_back(filePath + fileInfo.cFileName);
			}
			else
			{
				returnFileName.push_back(fileInfo.cFileName);
			}
		}
	}
}

#elif defined(__APPLE__) && defined(__MACH__)

#include <cstdio>
#include <dirent.h>

#include "filesystem.hpp"
namespace fs = ghc::filesystem;
#include <sys/syslimits.h>
#define MAX_PATH PATH_MAX

extern std::vector<std::string> openFileDialog(char const * const aTitle,
                        char const * const aDefaultPathAndFile,
                        const std::vector<std::string> & filters);

extern std::vector<std::string> saveFileDialog(char const * const aTitle,
                        char const * const aDefaultPathAndFile,
                        const std::vector<std::string> & filters);                        

/**
* @brief Opens a dialog box to save a file
* @param hwndOwner		urrent handle
* @param filename		Saved name from user
* @param FilterIndex	Index of chosen filter
* @return				Saved name if successful,
*						empty string otherwise
*/
std::string getADIFileName(void *hwndOwner, char* filename,  int & FilterIndex) {
    std::vector<std::string> filters;
    std::copy(std::begin(customFilters), std::end(customFilters), std::back_inserter(filters));
    std::vector<std::string> files = saveFileDialog("Select filename", filename, filters);

    if (files.size() == 0) {
        return "";
    }

    for (int ix = 0; ix < filters.size(); ix++) {
        if (files[0].substr(files[0].find_last_of(".") + 1) == filters[ix]) {
            FilterIndex = ix + 1;
            break;
        }
    }
    return files[0].substr(0, files[0].find_last_of(".")); //strip file extension
}

std::string openADIFileName(const char* filter, void *owner) {

    std::vector<std::string> filters;
    std::copy(std::begin(customFilters), std::end(customFilters), std::back_inserter(filters));
    fs::path curPath = fs::current_path();
    std::vector<std::string> files = openFileDialog("Select filename", curPath.c_str(), filters);

    if (files.size() == 0) {
        return "";
    }

    return files[0];
}

void getFilesList(std::string filePath, std::string extension, std::vector<std::string>& returnFileName, bool returnFullPath) {
	
	returnFileName.clear();

	DIR *dir = opendir(filePath.c_str());
	if (NULL == dir) {
		return;
	}

	//strip off '*' used in WIN32 APIs
	extension = extension.substr(1); 

	struct dirent * entry = readdir(dir);
	while (NULL != entry) {
		
		std::string file(entry->d_name);
		std::size_t pos = file.find_last_of('.');
		if (pos != std::string::npos && extension == file.substr(pos)) {
			if (returnFullPath) {
				returnFileName.push_back(filePath + "/" + file);
			}
			else {
				returnFileName.push_back(file);
			}
		}

		entry = readdir(dir);
	}

	closedir(dir);	
}

#elif defined(linux) || defined(__linux) || defined(__linux__)

#include <cstdio>
#include <dirent.h>

/**
* @brief Opens a dialog box to save a file
* @param hwndOwner		urrent handle
* @param filename		Saved name from user
* @param FilterIndex	Index of chosen filter
* @return				Saved name if successful,
*						empty string otherwise
*/
std::string getADIFileName(void *hwndOwner, char* filename,  int & FilterIndex)
{
    const char zenityP[] = "/usr/bin/zenity";
    char Call[2048];

    int ret = sprintf(Call,"%s  --file-selection --modal --title=\"%s\" ", zenityP, "Select filename");

    FILE *f = popen(Call,"r");
    std::fgets(filename, FILENAME_MAX, f);

    ret = pclose(f);
    if (ret < 0) {
        perror("file_name_dialog()");
    }

    return (ret == 0) ? filename : "";
}

std::string openADIFileName(const char* filter, void *owner) {

    const char zenityP[] = "/usr/bin/zenity";
    char Call[2048];
	char filename[FILENAME_MAX];

    int ret = sprintf(Call,"%s  --file-selection --modal --title=\"%s\" ", zenityP, "Select filename");

    FILE *f = popen(Call,"r");
    std::fgets(filename, FILENAME_MAX, f);

    ret = pclose(f);
    if (ret < 0) {
        perror("file_name_dialog()");
    }

    return (ret == 0) ? std::string(filename) : "";
}

void getFilesList(std::string filePath, std::string extension, std::vector<std::string>& returnFileName, bool returnFullPath) {
	
	returnFileName.clear();

	DIR *dir = opendir(filePath.c_str());
	if (NULL == dir) {
		return;
	}

	//strip off '*' used in WIN32 APIs
	extension = extension.substr(1); 

	struct dirent * entry = readdir(dir);
	while (NULL != entry) {
		
		std::string file(entry->d_name);
		std::size_t pos = file.find_last_of('.');
		if (pos != std::string::npos && extension == file.substr(pos)) {
			if (returnFullPath) {
				returnFileName.push_back(filePath + "/" + file);
			}
			else {
				returnFileName.push_back(file);
			}
		}

		entry = readdir(dir);
	}

	closedir(dir);	
}
#endif


