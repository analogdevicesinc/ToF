/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
#include "pybind11/functional.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"
#include "pybind11/numpy.h"

#include <fsf_common.h>

namespace py = pybind11;

PYBIND11_MODULE(fsf_python, m) {

    m.doc() = "ADI FSF uility python extensions";

    // FSF mode enum
    py::enum_<aditof::FsfMode>(m, "Mode")
        .value("READ", aditof::FsfMode::READ)
        .value("WRITE", aditof::FsfMode::WRITE);

    // Status definition enum
    py::enum_<aditof::FsfStatus>(m, "Status")
        .value("SUCCESS", aditof::FsfStatus::SUCCESS)
        .value("FILE_NOT_OPEN", aditof::FsfStatus::FILE_NOT_OPEN)
        .value("FILE_NOT_CREATED", aditof::FsfStatus::FILE_NOT_CREATED)
        .value("FILE_DOESNOT_EXIST", aditof::FsfStatus::FILE_DOESNOT_EXIST)
        .value("FILE_FORMAT_ERROR", aditof::FsfStatus::FILE_FORMAT_ERROR)
        .value("FILE_HEADER_ERROR", aditof::FsfStatus::FILE_HEADER_ERROR)
        .value("INVALID_OPERATION", aditof::FsfStatus::INVALID_OPERATION)
        .value("FAILED", aditof::FsfStatus::FAILED);

    // TODO: Add stream types enum

    // File header structure
    py::class_<aditof::FileHeader>(m, "FileHeader")
        .def(py::init<>())
        .def_readwrite("HeaderSize", &aditof::FileHeader::HeaderSize)
        .def_readwrite("MagicNumber", &aditof::FileHeader::MagicNumber)
        .def_readwrite("FileFormatMajorVersion", &aditof::FileHeader::FileFormatMajorVersion)
        .def_readwrite("FileFormatMinorVersion", &aditof::FileHeader::FileFormatMinorVersion)
        .def_readwrite("FileHeaderSize", &aditof::FileHeader::FileHeaderSize)
        .def_readwrite("StreamInfoSize", &aditof::FileHeader::StreamInfoSize)
        .def_readwrite("StreamHeaderSize", &aditof::FileHeader::StreamHeaderSize)
        .def_readwrite("OptionalFileHdrSize", &aditof::FileHeader::OptionalFileHdrSize)
        .def_readwrite("FileCommentSize", &aditof::FileHeader::FileCommentSize)
        .def_readwrite("FrameOffsetInfoLoc", &aditof::FileHeader::FrameOffsetInfoLoc)
        .def_readwrite("nFrames", &aditof::FileHeader::nFrames)
        .def_readwrite("nStreams", &aditof::FileHeader::nStreams);

    // aditof::Stream info structure
    py::class_<aditof::StreamInfo>(m, "StreamInfo")
        .def(py::init<>())
        .def_readwrite("SystemID", &aditof::StreamInfo::SystemID)
        .def_readwrite("StreamType", &aditof::StreamInfo::StreamType)
        .def_readwrite("ChannelFormat", &aditof::StreamInfo::ChannelFormat)
        .def_readwrite("BytesPerPixel", &aditof::StreamInfo::BytesPerPixel)
        .def_readwrite("nRowsPerStream", &aditof::StreamInfo::nRowsPerStream)
        .def_readwrite("nColsPerStream", &aditof::StreamInfo::nColsPerStream)
        .def_readwrite("OptionalStreamHdrSize", &aditof::StreamInfo::OptionalStreamHdrSize)
        .def_readwrite("StreamCommentSize", &aditof::StreamInfo::StreamCommentSize)
        .def_readwrite("CompressionScheme", &aditof::StreamInfo::CompressionScheme);

    // String wrapper
    struct optionalFileHeader {
        aditof::OptionalFileHeader string;
    };

    // Optional file header structure
    py::class_<optionalFileHeader>(m, "OptionalFileHeader")
        .def(py::init<>())
        .def_readwrite("string", &optionalFileHeader::string);

    // String wrapper
    struct fileComment {
        aditof::FileComment string;
    };

    // File comment structure
    py::class_<fileComment>(m, "FileComment")
        .def(py::init<>())
        .def_readwrite("string", &fileComment::string);

    // Stream structure
    py::class_<aditof::Stream>(m, "Stream")
        .def(py::init<>())
        .def_readwrite("streamHeader", &aditof::Stream::streamHeader)
        .def_readwrite("optionalStreamHeader", &aditof::Stream::optionalStreamHeader)
        .def_readwrite("streamComment", &aditof::Stream::streamComment)
        .def_readwrite("streamData", &aditof::Stream::streamData);

    // FSF class
    py::class_<aditof::FSF_Common>(m, "FSF_Common")
        .def(py::init<aditof::FsfMode>())

        .def("CreateFsfFile", &aditof::FSF_Common::CreateFsfFile, py::arg("pFilename"))

        .def("OpenFile", &aditof::FSF_Common::OpenFile, py::arg("pFilename"))

        .def("SaveFile", &aditof::FSF_Common::SaveFile)

        .def("CloseFile", &aditof::FSF_Common::CloseFile)

        .def("SetFileHeader", &aditof::FSF_Common::SetFileHeader, py::arg("fileHeader"))

        .def("GetFileHeader", &aditof::FSF_Common::GetFileHeader, py::arg("fileHeader"))

        .def("SetStreamInfo", &aditof::FSF_Common::SetStreamInfo,
            py::arg("streamIndex"), py::arg("streamInfo"))

        .def("GetStreamInfo", &aditof::FSF_Common::GetStreamInfo,
            py::arg("streamIndex"), py::arg("streamInfo"))

        .def("SetOptionalFileHeader",
             [](aditof::FSF_Common &fsf, optionalFileHeader &optFileHeader) {
                 return fsf.SetOptionalFileHeader(optFileHeader.string);
             },
             py::arg("optFileHeader"))

        .def("GetOptionalFileHeader",
             [](aditof::FSF_Common &fsf, optionalFileHeader &optFileHeader) {
                 return fsf.GetOptionalFileHeader(optFileHeader.string);
             },
             py::arg("optFileHeader"))

        .def("SetFileComment",
             [](aditof::FSF_Common &fsf, fileComment &fileComment) {
                 return fsf.SetFileComment(fileComment.string);
             },
             py::arg("fileComment"))

        .def("GetFileComment",
             [](aditof::FSF_Common &fsf, fileComment &fileComment) {
                 return fsf.GetFileComment(fileComment.string);
             },
             py::arg("fileComment"))

        .def("SetStream", &aditof::FSF_Common::SetStream, py::arg("frameIndex"),
            py::arg("streamIndex"), py::arg("stream"))

        .def("GetStream", &aditof::FSF_Common::GetStream, py::arg("frameIndex"),
            py::arg("streamIndex"), py::arg("stream"));
}
