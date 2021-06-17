/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
#pragma once
#ifndef TOF_CALIBRATION_TYPES_H
#define TOF_CALIBRATION_TYPES_H

#ifdef _WIN32
//disable zero-length array warning in MSVC
#pragma warning( push )
#pragma warning( disable : 4200 )
#endif

#include <stdint.h>

#define CALIB_FILE_VER_NEW_FORMAT 0x0301
#define CALIB_FILE_VER_NEW_FORMAT_V2 0x0303

/*
BLOCK ID Definitions
===================================
*/
#define CAL_BLOCK_ID_ADDRVAL_REGLIST  'A'
#define CAL_BLOCK_ID_DEVICE_CONFIG    'C'
#define CAL_BLOCK_ID_PIXEL_DEFECT     'D'
#define CAL_BLOCK_ID_FOI_MASK         'F'
#define CAL_BLOCK_ID_GEOMETRIC        'G'
#define CAL_BLOCK_ID_ILLUM_PROFILE    'I'
#define CAL_BLOCK_ID_LSDACS           'L'
#define CAL_BLOCK_ID_P0               'P'
#define CAL_BLOCK_ID_REGISTRATION     'R'
#define CAL_BLOCK_ID_SPATIAL_TEMPCOR  'S'
#define CAL_BLOCK_ID_TEMPCORRECTION   'T'
#define CAL_BLOCK_ID_RELATIVE_ILLUM   'V'
#define CAL_BLOCK_ID_COLUMN_DEFECT    '|'
#define CAL_BLOCK_ID_GAINCORRECTION   0xD7
#define CAL_BLOCK_ID_HDR              0xFF

#define CAL_HDR_V0_CONFIG_VER_STR_SIZE 128
#define CAL_HDR_V0_SERIAL_NUM_STR_SIZE 32
#define CAL_HDR_V0_CHIP_UNIQUE_ID_SIZE 32

//
// -------- Calibration File Layout -------
//
//      CAL_FILE_HEADER_V1
//      CAL_HEADER_BLOCK_V3
//      CAL_BLOCK_XXXX (XXXX Based on the type, every block subsequent to
//      CAL_FILE_HEADER_V1 starts with struct CAL_BLOCK_INFO_V1 - Typecast the
//      block based on BlockID indicated by struct CAL_BLOCK_INFO_V1 to parse
//      the contents) CAL_BLOCK_XXXX CAL_BLOCK_XXXX CAL_BLOCK_XXXX
// --------------------------------
//

// CCB File Format version 0x0303 Type definitions

struct CAL_BLOCK_INFO_V1
{
  uint8_t BlockID;
  uint8_t BlockVersion;
  uint16_t BlockCalibVer;
  uint32_t BlockSize;
  uint64_t CheckSum;
  uint16_t CalibrationYear;
  uint8_t CalibrationMonth;
  uint8_t CalibrationDay;
  uint8_t CalibrationHour;
  uint8_t CalibrationMinute;
  uint8_t CalibrationSecond;
  uint8_t CalibrationPass;
};

struct CAL_FILE_HEADER_V1
{
  uint16_t CalibFileVersion;
  uint16_t CalibVersion;
  uint16_t CalibHdrSize;
  uint8_t rsvd;
  uint8_t CalibFileHdrSize;
  uint32_t CalibFileSize;
  uint32_t rsvd1;
};

struct CAL_HEADER_BLOCK_V3
{
  struct CAL_BLOCK_INFO_V1 BlockInfo;
  uint16_t nBlocks;
  uint16_t ChipID;
  uint16_t nRows;
  uint16_t nCols;
  char ConfigVersionStr[CAL_HDR_V0_CONFIG_VER_STR_SIZE];
  char SerialNumber[CAL_HDR_V0_SERIAL_NUM_STR_SIZE];
  char ChipUniqueID[CAL_HDR_V0_CHIP_UNIQUE_ID_SIZE];
  uint32_t ControlAPIVersion;
  uint32_t FirmwareVersion;
  uint32_t CalibrationVersion;
  uint32_t SequenceVersion;
};

struct ADDRVAL_TUPLE
{
  uint16_t Address;
  uint16_t Value;
  uint16_t Mask;
  uint16_t Comment;
};

struct COLUMN_CRITERIA
{
  float ShortAmpVcm;
  float ShortADCVcm;
  float StdDevAmpVcm;
  uint32_t Thresholded;
};

struct TEMP_COEFFS
{
  float AmplitudeCoeff[8];
  float PhaseCoeff[8];
};

struct ROW_EXTENT
{
  uint16_t rowBegin;
  uint16_t rowEnd;
};

struct CAL_ADDRVAL_REG_BLOCK_V1
{
  struct CAL_BLOCK_INFO_V1 BlockInfo;

  uint16_t nTuples;
  uint16_t rsvd[3]; // (NLD) Changed size to 3 x uint16_t to match block size
  struct ADDRVAL_TUPLE Tuples[]; // Length: nTuples
};

struct CAL_LSDAC_BLOCK_V1
{
  struct CAL_BLOCK_INFO_V1 BlockInfo;

  uint16_t StartAddress;
  uint16_t nWrites;
  uint16_t ChecksumLSB;
  uint16_t ChecksumMSB;
  uint16_t Settings[]; // Length: nWrites
};

struct CAL_ILLUM_PROFILE_BLOCK_V2
{
  struct CAL_BLOCK_INFO_V1 BlockInfo;
  uint8_t Mode;
  uint8_t PixelBinning;
  uint8_t Laser;
  uint8_t rsvd;

  uint32_t ProfileDataSize;
  float ABScale;
  float TargetRefl; // Albedo
  int16_t SensorToLSOffsetX;
  int16_t SensorToLSOffsetY;
  int16_t SensorToLSOffsetZ;
  int16_t TargetDist;
  uint16_t nRows;
  uint16_t nCols;
  uint16_t nOffsetRows;
  uint16_t nOffsetCols;
  uint8_t ProfileData[]; // Placeholder to read the Profiledata (PNG Data) whose
  // size is ProfileDataSize
};

struct CAL_FOI_MASK_BLOCK_V0_V1INFO
{
  struct CAL_BLOCK_INFO_V1 BlockInfo;

  uint8_t Mode;
  uint8_t PixelBinning;
  uint8_t rsvd[6];
  uint16_t nRows;
  uint16_t nCols;
  uint16_t nOffsetRows;
  uint16_t nOffsetCols;
  struct ROW_EXTENT RowExtents[]; // Length of nRows
};

struct CAL_COLUMN_DEFECT_BLOCK_V1
{
  struct CAL_BLOCK_INFO_V1 BlockInfo;

  uint16_t nCols;
  uint16_t ChipQuality;
  uint16_t TotalBadStripeWidth;
  uint16_t rsvd;
  float Thresholds[32];
  struct COLUMN_CRITERIA ColCriteria[]; // Length of nCols
};

struct CAL_GEOMETRIC_BLOCK_V3
{
  struct CAL_BLOCK_INFO_V1 BlockInfo;

  uint16_t nRows;
  uint16_t nCols;
  uint16_t offsetRows;
  uint16_t offsetCols;
  uint8_t mode;
  uint8_t pixelBinning;
  uint16_t rsvd;

  uint32_t CameraModel;
  float Fc1; // Focal length in X direction
  float Fc2; // Focal length in Y direction
  float cc1; // Principal point X coordinate
  float cc2; // Principal point Y coordinate
  float Cx;  // Center of Distortion
  float Cy;
  float Kc1; // Distortion coeff Kc1, kc2, kc3, kc4, kc5
  float Kc2;
  float Kc3;
  float Kc4;
  float Kc5;
  float Kc6;
  float Tx; // Tangential Distortion
  float Ty;
};

// template <uint32_t rows, uint32_t cols> struct CAL_P0BLOCK_V0_V1INFO {
//   struct CAL_BLOCK_INFO_V1 BlockInfo;
//   uint8_t Mode;
//   uint8_t FreqIndex;
//   uint16_t Freq;
//   uint16_t IntegrationTime;
//   uint16_t ABMultiplier;
//   uint16_t nRows;
//   uint16_t nCols;
//   uint16_t OffsetRows;
//   uint16_t OffsetCols;
//   uint16_t P0_Table[rows * cols];
//   uint16_t T0Coeffs[2];
// };

struct CAL_P0BLOCK_V4
{
  struct CAL_BLOCK_INFO_V1 BlockInfo;
  uint8_t Mode;
  uint8_t FreqIndex;
  uint16_t Freq;
  uint16_t IntegrationTime;

  uint8_t nRowsCoeffs;
  uint8_t nColCoeffs;
  uint8_t AnalogBinRows;
  uint8_t AnalogBinCols;
  uint8_t DigitalBinRows;
  uint8_t DigitalBinCols;
  uint8_t SubSamplingRows;
  uint8_t SubSamplingCols;
  uint8_t SoftwareBinRows;
  uint8_t SoftwareBinCols;

  uint16_t nRows;
  uint16_t nCols;
  uint16_t OffsetRows;
  uint16_t OffsetCols;
  float P0Coeffs[]; // Row Coeffs (nRows * nRowCoeffs) + Pad (if nRows are odd)
  // + Col Coeffs (nCols * nColCoeffs) + Pad
};

struct CAL_TEMP_CORR_BLOCK_V0_V1INFO
{
  struct CAL_BLOCK_INFO_V1 BlockInfo;
  uint8_t Mode;
  uint8_t FreqIndex;
  uint8_t CaptureNum;
  uint8_t LaserID;
  uint16_t rsvd[2];
  struct TEMP_COEFFS coeffs;
};

// Gain Correction Block Definitions
struct GLOBAL_ADC_SETTINGS
{
  uint16_t IRamp;
  uint16_t UpdnoOffset;
};

struct GAIN_COMPARATOR
{
  uint8_t Vref1Dac;
  uint8_t Vref1Set;
  uint8_t Vref2Dac;
  uint8_t Vref2Set;
  uint8_t Vref3Dac;
  uint8_t Vref3Set;
  uint8_t VCMDac;
  uint8_t VCMSet;
};

struct COLUMN_GAIN_CORRECTION
{
  uint16_t A1D1[4096];
  uint16_t A1D2[1024];
  uint16_t A1D4[512];
  uint16_t A2D1[4096];
  uint16_t A2D2[1024];
  uint16_t A4D1[2048];
};

struct CAL_GAIN_CORRECTION_BLOCK
{
  struct CAL_BLOCK_INFO_V1 BlockInfo;
  uint16_t GainCalConfigVersion;
  uint16_t SubsamplingDataPresent;
  struct GLOBAL_ADC_SETTINGS ADC;
  struct GAIN_COMPARATOR GainComparator;
  uint16_t InverseGlobalADCGain[4];
  struct COLUMN_GAIN_CORRECTION PerColGainAdjustment;
  struct COLUMN_GAIN_CORRECTION PerColOffsetAdjustment;
};

// Available per frequency
struct CAL_SPATIAL_TEMP_CORR_BLOCK
{
  struct CAL_BLOCK_INFO_V1 BlockInfo;
  uint8_t Mode;
  uint8_t FreqIndex;
  uint16_t Freq;
  uint16_t IntegrationTime;
  uint16_t CalibrationPhase;
  uint16_t nRows;
  uint16_t nCols;
  uint16_t nOffsetRows;
  uint16_t nOffsetCols;
  float SimpleRowModel[2];
  float SimpleColModel[5];
  uint16_t NumberOfCalib;
  uint16_t rsvd;
  float RowTemperatureCoeff[4];
  float ColTemperatureCoeff[10];
  float BaseCalTemperature[3];
  float rsvd2;
  uint16_t PerColOffsets[]; // Number of elements called out by nCols field +
  // Padding (if number of colums are odd)
};

struct CAL_RELATIVE_ILLUM_BLOCK
{
  struct CAL_BLOCK_INFO_V1 BlockInfo;
  uint8_t Mode;
  uint8_t PixelBinning;
  uint16_t rsvd;
  uint16_t nSamples;
  uint16_t SampleSpacing;
  float ScaleFactor;
  float ClampValue;
  uint16_t CentroidX;
  uint16_t CentroidY;
  float TiltX;
  float TiltY;
  uint16_t Model[]; // Number of elements called out by nSamples field + Padding
  // (if number of samples are odd)
};

#ifdef _WIN32
#pragma warning( pop )
#endif

#endif // TOF_CALIBRATION_TYPES_H
