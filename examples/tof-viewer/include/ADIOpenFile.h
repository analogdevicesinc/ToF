/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
#include <windows.h>
#include <string.h>
#include <iostream>
#include <string_view>
#include <Commdlg.h>
using namespace std;


char* customFilter = "All Files (*.*)\0*.*\0";

/**
* @brief		Returns an empty string if cancelled.
*				Opens a dialog box to fetch a file with 
*				a custom file extension
* @param filter	Is assigned a customized filter per
*				camera requirements
* @param owner	NULL
* @return		Selected file name with its extension
*/
string openADIFileName(char* filter = customFilter, HWND owner = NULL)
{
	OPENFILENAME file;
	char fileName[MAX_PATH] = "";
	ZeroMemory(&file, sizeof(file));
	file.lStructSize = sizeof(OPENFILENAME);
	file.hwndOwner = owner;
	file.lpstrFilter = filter;
	file.lpstrFile = fileName;
	file.nMaxFile = MAX_PATH;
	file.Flags = OFN_EXPLORER | OFN_FILEMUSTEXIST | OFN_HIDEREADONLY;
	file.lpstrDefExt = "";
	string fileNameStr;
	if (GetOpenFileName(&file))
		fileNameStr = fileName;
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
string getADIFileName(HWND hwndOwner, char* filename,  int & FilterIndex)
{
	OPENFILENAME ofn = { 0 };
	ofn.lStructSize = sizeof(ofn);
	ofn.Flags = OFN_EXPLORER | OFN_PATHMUSTEXIST;
	ofn.hInstance = GetModuleHandle(0);
	ofn.hwndOwner = hwndOwner;
	ofn.nMaxFile = MAX_PATH;
	ofn.lpstrFile = filename;
	ofn.nFilterIndex = 1;
	ofn.lpstrFilter = customFilter;
	ofn.lpstrDefExt = "";
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
