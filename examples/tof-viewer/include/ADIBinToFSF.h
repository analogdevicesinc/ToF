/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
#ifndef ADIBINTOFSF_H
#define ADIBINTOFSF_H

#include "ADIToFRecorder.h"
#include <glog/logging.h>
#include <functional>

class ADIBinToFSF
{
public:

	/**
	* @brief ADI BinToFSF constructor
	*/
	ADIBinToFSF();

	/**
	* @brief ADI BinToFSF destructor
	*/
	~ADIBinToFSF();

	/**
	* @brief Converts a point cloud ONLY file with *.bin extension
	*        into an FSF file
	*/
	aditof::FSF* ConvertPointCloudToFSF(const std::string& fileName, int& frames, int& width, int& height);

private:

};

#endif // ADIBINTOFSF_H
