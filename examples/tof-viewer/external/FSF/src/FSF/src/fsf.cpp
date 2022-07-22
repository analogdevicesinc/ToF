/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
#include <fsf/fsf.h>

namespace aditof {

FSF::FSF() {}

FSF::FSF(FsfMode) {}

FSF::~FSF() {}

FsfStatus FSF::CreateFsfFile(const char *) {
    return FsfStatus::FAILED;
}

FsfStatus FSF::OpenFile(const char *) {
    return FsfStatus::FAILED;
}

FsfStatus FSF::SaveFile(void) {
    return FsfStatus::FAILED;
}

FsfStatus FSF::CloseFile(void) {
    return FsfStatus::FAILED;
}

FsfStatus FSF::SetFileHeader(FileHeader &) {
    return FsfStatus::FAILED;
}

FsfStatus FSF::GetFileHeader(FileHeader &) {
    return FsfStatus::FAILED;
}

FsfStatus FSF::SetStreamInfo(const unsigned int, StreamInfo &) {
    return FsfStatus::FAILED;
}

FsfStatus FSF::GetStreamInfo(const unsigned int, StreamInfo &) {
    return FsfStatus::FAILED;
}

FsfStatus FSF::SetOptionalFileHeader(OptionalFileHeader &) {
    return FsfStatus::FAILED;
}

FsfStatus FSF::GetOptionalFileHeader(OptionalFileHeader &) {
    return FsfStatus::FAILED;
}

FsfStatus FSF::SetFileComment(FileComment &) {
    return FsfStatus::FAILED;
}

FsfStatus FSF::GetFileComment(FileComment &) {
    return FsfStatus::FAILED;
}

FsfStatus FSF::SetStream(const unsigned int, const unsigned int, Stream &) {
    return FsfStatus::FAILED;
}

FsfStatus FSF::GetStream(const unsigned int, const unsigned int, Stream &) {
    return FsfStatus::FAILED;
}

} // namespace aditof
