/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
#ifndef ADI_FSF_OPTIONS_H
#define ADI_FSF_OPTIONS_H
#include "ADIImGUIExtensions.h"

#ifdef ADI_FSF_OPTIONS_INLINE
#define ADI_FSF_OPTIONS_H_FUNC static
#else
#define ADI_FSF_OPTIONS_H_FUNC static inline
#endif



aditof::FSF* OptionspFsfRead;
aditof::FileHeader optfileHeader;
aditof::StreamInfo optstreamInfo;
FSFStreamEnable checkAvailability;

/**
*@brief Returns the proper boolean logic to enable/disable point cloud data
*/
ADI_FSF_OPTIONS_H_FUNC void pointCloudEnable(bool _isX, bool _isY, bool _isDepth, bool& xEnable, bool& yEnable, bool& depthEnable);

/**
* @brief Write FSF file, set stream options
*/
ADI_FSF_OPTIONS_H_FUNC void SetFSFStreamInfo(int mainWindowWidth, int mainWindowHeight, bool& showWindow,
											 bool& startFSFRec, FSFStreamEnable& _streamEnable)
{
	//Set FSF Options window current position. This is in fucntion of the main window height and width
	ImGui::SetNextWindowPos({ (float)mainWindowWidth / 4 , (float)mainWindowHeight / 4 });
	//This is known widow size with nStreams = 19
	//ImVec2 winSize = { 550.0, 650.0 };//For full option spectrum
	ImVec2 winSize = { 550.0, (float)(200.0 + 80.0 * 2) };
	ImGui::SetNextWindowSize(winSize);
	ImGui::Begin("FSF Options", NULL, ImGuiWindowFlags_NoResize);
	bool _isX;
	bool _isY;
	bool _isDepth;
	ImGui::SetNextTreeNodeOpen(true, ImGuiCond_FirstUseEver);

	if (ImGui::TreeNode("FSF Stream Type Options"))
	{
		ImGui::NewLine();
		ImGui::SameLine();
		adiMainWindow::ImGuiExtensions::ADICheckbox("ACTIVE_BR", &_streamEnable.active_br, true);
		ImGui::SameLine();
		_isDepth = adiMainWindow::ImGuiExtensions::ADICheckbox("DEPTH", &_streamEnable.depth, true);
		ImGui::NewLine();
		ImGui::SameLine();
		/*adiMainWindow::ImGuiExtensions::ADICheckbox("COMMON_MODE", &_streamEnable.common_mode, false);
		ImGui::SameLine();
		adiMainWindow::ImGuiExtensions::ADICheckbox("CONF", &_streamEnable.conf, false);
		ImGui::NewLine();
		ImGui::SameLine();
		adiMainWindow::ImGuiExtensions::ADICheckbox("PHASE", &_streamEnable.phase, false);
		ImGui::SameLine();
		adiMainWindow::ImGuiExtensions::ADICheckbox("RADIAL", &_streamEnable.radial, false);
		ImGui::NewLine();
		ImGui::SameLine();
		adiMainWindow::ImGuiExtensions::ADICheckbox("RADIAL_FILT", &_streamEnable.radial_filt, false);
		ImGui::SameLine();
		adiMainWindow::ImGuiExtensions::ADICheckbox("RAW", &_streamEnable.raw, false);
		ImGui::NewLine();
		ImGui::SameLine();
		adiMainWindow::ImGuiExtensions::ADICheckbox("RAW_IMAG", &_streamEnable.raw_imag, false);
		ImGui::SameLine();
		adiMainWindow::ImGuiExtensions::ADICheckbox("RAW_IMAG_FILT", &_streamEnable.raw_imag_filt, false);
		ImGui::NewLine();
		ImGui::SameLine();
		adiMainWindow::ImGuiExtensions::ADICheckbox("RAW_NORM", &_streamEnable.raw_norm, false);
		ImGui::SameLine();
		adiMainWindow::ImGuiExtensions::ADICheckbox("RAW_REAL", &_streamEnable.raw_real, false);
		ImGui::NewLine();
		ImGui::SameLine();
		adiMainWindow::ImGuiExtensions::ADICheckbox("RAW_REAL_FILT", &_streamEnable.raw_real_filt, false);
		ImGui::SameLine();
		adiMainWindow::ImGuiExtensions::ADICheckbox("REFLECTIVITY", &_streamEnable.reflectivity, false);
		ImGui::NewLine();
		ImGui::SameLine();
		adiMainWindow::ImGuiExtensions::ADICheckbox("RGB", &_streamEnable.rgb, false);
		ImGui::SameLine();
		adiMainWindow::ImGuiExtensions::ADICheckbox("VARIANCE", &_streamEnable.variance, false);
		ImGui::NewLine();
		ImGui::SameLine();	*/	
		_isX = adiMainWindow::ImGuiExtensions::ADICheckbox("X", &_streamEnable.x, true);
		ImGui::SameLine();
		_isY = adiMainWindow::ImGuiExtensions::ADICheckbox("Y", &_streamEnable.y, true);
		ImGui::NewLine();
		ImGui::SameLine();
		/*adiMainWindow::ImGuiExtensions::ADICheckbox("UNKNOWN", &_streamEnable.unknown, false);
		ImGui::NewLine();*/
		ImGui::TreePop();
		pointCloudEnable(_isX, _isY, _isDepth, _streamEnable.x, _streamEnable.y, _streamEnable.depth);
	}
	ImGui::Separator();
	if (adiMainWindow::ImGuiExtensions::ADIButton("Ok", true))
	{
		showWindow = false;
		startFSFRec = true;
	}
	ImGui::SameLine();
	if (adiMainWindow::ImGuiExtensions::ADIButton("Cancel", true))
	{
		FSFStreamEnable resetStream;
		_streamEnable = resetStream;
		showWindow = false;
		startFSFRec = false;
	}

	ImGui::End();
}

/**
* @brief Read from FSF file, will fetch the options from a given file.
*/
ADI_FSF_OPTIONS_H_FUNC void GetFSFInfo(const char* pFilename, FSFStreamEnable _checkAvailability)
{
	OptionspFsfRead = new aditof::FSF_Common{ aditof::FsfMode::READ, FSF_CUSTOM_MAX_NUM_FRAMES };
	OptionspFsfRead->OpenFile(pFilename);
	//Get File Header
	OptionspFsfRead->GetFileHeader(optfileHeader);
	checkAvailability = _checkAvailability;
}

/**
*@brief Read from FSF file, will show the options the read file has to offer
*/
ADI_FSF_OPTIONS_H_FUNC void ShowFSFOptionsGUI(int mainWindowWidth, int mainWindowHeight, bool &showWindow, bool &startFSFPb, 
											  FSFStreamEnable &_streamEnable)
{	
	////Set FSF Options window current position. This is in fucntion of the main window height and width
	//ImGui::SetNextWindowPos({ (float)mainWindowWidth / 4 , (float)mainWindowHeight / 4 });
	////Dynamic window size in fucntion of nStreams value. The larger nStreams is, the bigger the window will be
	//ImVec2 winSize = { 550.0, (float)(200.0 + 80.0 * (optfileHeader.nStreams / 2) )};
	//ImGui::SetNextWindowSize(winSize);
	//ImGui::Begin("FSF Options", NULL, ImGuiWindowFlags_NoResize);
	//ImGui::SetNextTreeNodeOpen(true, ImGuiCond_FirstUseEver);
	bool _isX;
	bool _isY;
	bool _isDepth;
	//if (ImGui::TreeNode("FSF Stream Type Options"))
	{
		if (OptionspFsfRead != nullptr)
		{
			for (int strCtr = 0; strCtr < optfileHeader.nStreams; strCtr++)
			{
				OptionspFsfRead->GetStreamInfo(strCtr, optstreamInfo);

				/*aditof::StreamType sTyp = (aditof::StreamType)optstreamInfo.StreamType;
				switch (sTyp)
				{
					case aditof::StreamType::STREAM_TYPE_ACTIVE_BR:
						if (!checkAvailability.active_br)
						{
							_streamEnable.active_br = true;
							checkAvailability.active_br = true;
						}
						adiMainWindow::ImGuiExtensions::ADICheckbox("ACTIVE_BR", &_streamEnable.active_br, true);
						
						break;
					case aditof::StreamType::STREAM_TYPE_COMMON_MODE:
						adiMainWindow::ImGuiExtensions::ADICheckbox("COMMON_MODE", &_streamEnable.common_mode, false);
						break;
					case aditof::StreamType::STREAM_TYPE_CONF:
						adiMainWindow::ImGuiExtensions::ADICheckbox("CONF", &_streamEnable.conf, false);
						break;
					case aditof::StreamType::STREAM_TYPE_DEPTH:
						if (!checkAvailability.depth)
						{
							_streamEnable.depth = true;
							checkAvailability.depth = true;
						}
						_isDepth = adiMainWindow::ImGuiExtensions::ADICheckbox("DEPTH", &_streamEnable.depth, true);
						break;
					case aditof::StreamType::STREAM_TYPE_PHASE:
						adiMainWindow::ImGuiExtensions::ADICheckbox("PHASE", &_streamEnable.phase, false);
						break;
					case aditof::StreamType::STREAM_TYPE_RADIAL:
						adiMainWindow::ImGuiExtensions::ADICheckbox("RADIAL", &_streamEnable.radial, false);
						break;
					case aditof::StreamType::STREAM_TYPE_RADIAL_FILT:
						adiMainWindow::ImGuiExtensions::ADICheckbox("RADIAL_FILT", &_streamEnable.radial_filt, false);
						break;
					case aditof::StreamType::STREAM_TYPE_RAW:
						adiMainWindow::ImGuiExtensions::ADICheckbox("RAW", &_streamEnable.raw, false);
						break;
					case aditof::StreamType::STREAM_TYPE_RAW_IMAG:
						adiMainWindow::ImGuiExtensions::ADICheckbox("RAW_IMAG", &_streamEnable.raw_imag, false);
						break;
					case aditof::StreamType::STREAM_TYPE_RAW_IMAG_FILT:
						adiMainWindow::ImGuiExtensions::ADICheckbox("RAW_IMAG_FILT", &_streamEnable.raw_imag_filt, false);
						break;
					case aditof::StreamType::STREAM_TYPE_RAW_NORM:
						adiMainWindow::ImGuiExtensions::ADICheckbox("RAW_NORM", &_streamEnable.raw_norm, false);
						break;
					case aditof::StreamType::STREAM_TYPE_RAW_REAL:
						adiMainWindow::ImGuiExtensions::ADICheckbox("RAW_REAL", &_streamEnable.raw_real, false);
						break;
					case aditof::StreamType::STREAM_TYPE_RAW_REAL_FILT:
						adiMainWindow::ImGuiExtensions::ADICheckbox("RAW_REAL_FILT", &_streamEnable.raw_real_filt, false);
						break;
					case aditof::StreamType::STREAM_TYPE_REFLECTIVITY:
						adiMainWindow::ImGuiExtensions::ADICheckbox("REFLECTIVITY", &_streamEnable.reflectivity, false);
						break;
					case aditof::StreamType::STREAM_TYPE_RGB:
						adiMainWindow::ImGuiExtensions::ADICheckbox("RGB", &_streamEnable.rgb, false);
						break;
					case aditof::StreamType::STREAM_TYPE_UNKNOWN:
						adiMainWindow::ImGuiExtensions::ADICheckbox("UNKNOWN", &_streamEnable.unknown, false);
						break;
					case aditof::StreamType::STREAM_TYPE_VARIANCE:
						adiMainWindow::ImGuiExtensions::ADICheckbox("VARIANCE", &_streamEnable.variance, false);
						break;
					case aditof::StreamType::STREAM_TYPE_X:	
							if (!checkAvailability.x)
							{
								_streamEnable.x = true;
								checkAvailability.x = true;
							}
							_isX = adiMainWindow::ImGuiExtensions::ADICheckbox("X", &_streamEnable.x, true);
						break;
					case aditof::StreamType::STREAM_TYPE_Y:	
							if (!checkAvailability.y)
							{
								_streamEnable.y = true;
								checkAvailability.y = true;
							}
							_isY = adiMainWindow::ImGuiExtensions::ADICheckbox("Y", &_streamEnable.y, true);
						break;					
					default:
						adiMainWindow::ImGuiExtensions::ADICheckbox("UNKNOWN", &_streamEnable.unknown, true);
						break;
				}*/
				
				//if(strCtr %2 == 0)//Make two columns
				//{ 
				//	ImGui::SameLine();
				//}
				//else
				//{
				//	ImGui::NewLine();
				//}
			}
			//ImGui::NewLine();
		}
		
		//ImGui::TreePop();
	}
	//ImGui::Separator();	
	//if (adiMainWindow::ImGuiExtensions::ADIButton("Ok", true))
	//{
		showWindow = false;
		OptionspFsfRead->CloseFile();
		startFSFPb = true;
	//}
	//ImGui::SameLine();
	//if (adiMainWindow::ImGuiExtensions::ADIButton("Cancel", true))
	//{
	//	showWindow = false;
	//	startFSFPb = false;
	//	OptionspFsfRead->CloseFile();
	//}
	//ImGui::End();
}

ADI_FSF_OPTIONS_H_FUNC void pointCloudEnable(bool _isX, bool _isY, bool _isDepth, bool& xEnable, bool& yEnable, bool& depthEnable)
{
	if (_isX)
	{
		yEnable = xEnable;
		if (xEnable && !depthEnable)
		{
			depthEnable = true;
		}
	}
	if (_isY)
	{
		xEnable = yEnable;
		if (yEnable && !depthEnable)
		{
			depthEnable = true;
		}
	}
	if (_isDepth)
	{
		if (!depthEnable && (xEnable || yEnable))
		{
			xEnable = false;
			yEnable = false;
		}
	}
}

#endif // ADI_FSF_OPTIONS_H