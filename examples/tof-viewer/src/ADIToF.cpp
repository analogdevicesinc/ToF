/********************************************************************************/
/*                                                                              */
/* Copyright (c) Microsoft Corporation. All rights reserved.					*/
/*  Portions Copyright (c) 2020 Analog Devices Inc.								*/
/* Licensed under the MIT License.												*/
/*																				*/
/********************************************************************************/

// ADIToFTest.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include <cctype>
#include <iostream>
#include "system.h"
#include <glog/logging.h>

#include "ADIMainWindow.h"

ADIViewerArgs ProcessArgs(int argc, char** argv);

ADIViewerArgs ProcessArgs(int argc, char** argv)
{
	ADIViewerArgs args;

	// Skip argv[0], which is the path to the executable
	//
	for (int i = 1; i < argc; i++)
	{
		// Force to uppercase
		//
		std::string arg = argv[i];
		std::transform(arg.begin(), arg.end(), arg.begin(), [](unsigned char c) {
			return static_cast<unsigned char>(std::toupper(c));
			});

		if (arg == "-HIGHDPI")
		{
			args.HighDpi = true;
		}
		else if (arg == "-NORMALDPI")
		{
			args.HighDpi = false;
		}
	}

	return args;
}

int main(int argc, char **argv)
{
	auto view = std::make_shared<adiMainWindow::ADIMainWindow>();//Create a new instance
	
	if (view->startImGUI(ProcessArgs(argc, argv)))
	{
		view->render();
	}
	return 0; 
}