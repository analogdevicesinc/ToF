/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
#ifndef FSF_H
#define FSF_H

#include <fsf/fsf_definitions.h>

namespace aditof {

class FSF
{
    public:
        FSF();

        /**
         * @brief   FSF Constructor.
         *
         * @details Create an FSF file utility object in the given mode.
         *
         * @param   mode - FSF file utility in READ or WRITE mode
         */
        FSF(FsfMode mode);

        /**
         * @brief   FSF Destructor.
         */
        virtual ~FSF();

        /**
         * @brief   Create an FSF file.
         *
         * @details This creates a new file with the specified file name for
         *          writing internally allocated FSF data structures. Internal
         *          structures are created through the corresponding Setxxx
         *          methods.
         *
         * @param   pFilename - Name of the FSF file to create
         *
         * @return  Status
         *              - SUCCESS               If able to successfully create
         *                                      a new file
         *              - FILE_NOT_CREATED      If the file cannot be created
         *                                      with the given name
         *              - INVALID_OPERATION     If API is invoked with the FSF
         *                                      file utility in READ mode
         */
        virtual FsfStatus CreateFsfFile(const char *pFilename);

        /**
         * @brief   Open an FSF file for reading.
         *
         * @details The specified FSF file is opened for reading. Contents of
         *          the file can be read into allocated FSF data structures
         *          through the corresponding Getxxx methods.
         *
         * @param   pFilename - Name of the FSF file to read
         *
         * @return  Status
         *              - SUCCESS               If able to successfully open
         *                                      the file
         *              - FILE_DOESNOT_EXIST    If tried to open a file that
         *                                      does not exist
         *              - FILE_FORMAT_ERROR     If the opened file is of invalid
         *                                      format
         *              - INVALID_OPERATION     If API is invoked with the FSF
         *                                      file utility in WRITE mode
         */
        virtual FsfStatus OpenFile(const char *pFilename);

        /**
         * @brief   Save to an FSF file.
         *
         * @details This populates the created FSF file with the contents of
         *          the internally allocated FSF data structures.
         *
         * @return  Status
         *              - SUCCESS               If able to successfully save FSF
         *                                      contents to file
         *              - FILE_HEADER_ERROR     If the file header structure is
         *                                      invalid
         *              - FILE_NOT_OPEN         If file is not created before
         *                                      saving contents to file
         *              - FAILED                If there are errors while
         *                                      saving structures into file
         *              - INVALID_OPERATION     If API is invoked with the FSF
         *                                      file utility in READ mode
         */
        virtual FsfStatus SaveFile(void);

        /**
         * @brief   Close the created/opened FSF file.
         *
         * @return  Status
         *              - SUCCESS               Always returns successfully
         */
        virtual FsfStatus CloseFile(void);

        /**
         * @brief   Set FSF FileHeader structure.
         *
         * @details The internally allocated FileHeader structure is populated
         *          with the contents of the referenced FileHeader structure.
         *
         * @return  Status
         *              - SUCCESS               If internal structures are
         *                                      updated successfully
         *              - FILE_HEADER_ERROR     If the file header structure is
         *                                      invalid
         *              - INVALID_OPERATION     If API is invoked with the FSF
         *                                      file utility in READ mode
         */
        virtual FsfStatus SetFileHeader(FileHeader &fileHeader);

        /**
         * @brief   Get FileHeader information.
         *
         * @details The referenced FileHeader structure is populated with the
         *          file header information from the opened FSF file.
         *
         * @return  Status
         *              - SUCCESS               If the file header is retrieved
         *                                      successfully
         *              - FILE_NOT_OPEN         If tried to read file header
         *                                      with the file not open
         *              - FAILED                If there are errors encountered
         *                                      while reading from file
         *              - FILE_HEADER_ERROR     If the retrieved file header is
         *                                      invalid
         *              - INVALID_OPERATION     If API is invoked with the FSF
         *                                      file utility in WRITE mode
         */
        virtual FsfStatus GetFileHeader(FileHeader &fileHeader);

        /**
         * @brief   Set StreamInfo structure for the indexed stream.
         *
         * @details The internally allocated StreamInfo structure for the
         *          selected stream is populated with the contents of the
         *          referenced StreamInfo structure. The FileHeader must be
         *          set first prior to setting the StreamInfo structures.
         *
         * @return  Status
         *              - SUCCESS               If internal structures are
         *                                      updated successfully
         *              - FILE_HEADER_ERROR     If the file header structure is
         *                                      invalid
         *              - FAILED                If the stream info structure
         *                                      index is invalid
         *              - INVALID_OPERATION     If API is invoked with the FSF
         *                                      file utility in READ mode
         */
        virtual FsfStatus SetStreamInfo(const unsigned int streamIndex,
                                        StreamInfo &streamInfo);

        /**
         * @brief   Get StreamInfo information.
         *
         * @details The referenced StreamInfo structure is populated with the
         *          stream information for the selected stream from the opened
         *          FSF file. The FileHeader must be retrieved first prior to
         *          getting the stream information.
         *
         * @return  Status
         *              - SUCCESS               If the stream info is retrieved
         *                                      successfully
         *              - FILE_HEADER_ERROR     If the file header structure is
         *                                      invalid
         *              - FILE_NOT_OPEN         If tried to read stream info
         *                                      with the file not open
         *              - FAILED                If there are errors encountered
         *                                      while reading from file or the
         *                                      stream info structure index is
         *                                      invalid
         *              - INVALID_OPERATION     If API is invoked with the FSF
         *                                      file utility in WRITE mode
         */
        virtual FsfStatus GetStreamInfo(const unsigned int streamIndex,
                                        StreamInfo &streamInfo);

        /**
         * @brief   Set OptionalFileHeader string.
         *
         * @details If the OptionalStreamHdrSize in the FileHeader is non-zero,
         *          a buffer enough to hold the OptionalFileHeader string is
         *          internally allocated. The allocated buffer is populated with
         *          the contents of the referenced OptionalFileHeader string.
         *          The FileHeader must be set first prior to setting the
         *          OptionalFileHeader string.
         *
         * @return  Status
         *              - SUCCESS               If internal structures are
         *                                      updated successfully
         *              - FILE_HEADER_ERROR     If the file header structure is
         *                                      invalid
         *              - FAILED                If OptionalFileHdrSize is different
         *                                      with actual string size
         *              - INVALID_OPERATION     If API is invoked with the FSF
         *                                      file utility in READ mode
         */
        virtual FsfStatus SetOptionalFileHeader(OptionalFileHeader &optFileHeader);

        /**
         * @brief   Get OptionalFileHeader string.
         *
         * @details If the OptionalStreamHdrSize in the FileHeader is non-zero,
         *          the referenced OptionalFileHeader string buffer is populated
         *          with the contents of the OptionalFileHeader string from the
         *          opened FSF file. The FileHeader must be retrieved first
         *          prior to getting the OptionalFileHeader string.
         *
         * @return  Status
         *              - SUCCESS               If the optional file header
         *                                      is retrieved successfully
         *              - FILE_HEADER_ERROR     If the file header structure is
         *                                      invalid
         *              - FILE_NOT_OPEN         If tried to read optional file
         *                                      header with the file not open
         *              - FAILED                If there are errors encountered
         *                                      while reading from file
         *              - INVALID_OPERATION     If API is invoked with the FSF
         *                                      file utility in WRITE mode
         */
        virtual FsfStatus GetOptionalFileHeader(OptionalFileHeader &optFileHeader);

        /**
         * @brief   Set FileComment string.
         *
         * @details If the FileCommentSize in the FileHeader is non-zero, a
         *          buffer enough to hold the FileComment string is internally
         *          allocated. The allocated buffer is populated with the
         *          contents of the referenced FileComment string. The
         *          FileHeader must be set first prior to setting the
         *          FileComment string.
         *
         * @return  Status
         *              - SUCCESS               If internal structures are
         *                                      updated successfully
         *              - FILE_HEADER_ERROR     If the file header structure is
         *                                      invalid
         *              - FAILED                If FileCommentSize is different
         *                                      with actual string size
         *              - INVALID_OPERATION     If API is invoked with the FSF
         *                                      file utility in READ mode
         */
        virtual FsfStatus SetFileComment(FileComment &fileComment);

        /**
         * @brief   Get FileComment string.
         *
         * @details If the FileCommentSize in the FileHeader is non-zero, the
         *          referenced FileComment string buffer is populated with the
         *          contents of the FileComment string from the opened FSF file.
         *          The FileHeader must be retrieved first prior to getting the
         *          FileComment string.
         *
         * @return  Status
         *              - SUCCESS               If the file comment string
         *                                      is retrieved successfully
         *              - FILE_HEADER_ERROR     If the file header structure is
         *                                      invalid
         *              - FILE_NOT_OPEN         If tried to read file comment
         *                                      with the file not open
         *              - FAILED                If there are errors encountered
         *                                      while reading from file
         *              - INVALID_OPERATION     If API is invoked with the FSF
         *                                      file utility in WRITE mode
         */
        virtual FsfStatus GetFileComment(FileComment &fileComment);

        /**
         * @brief   Set Streams.
         *
         * @details The internally allocated Stream structure for the selected
         *          stream is populated with the contents of the referenced
         *          Stream structure. Stream structure are structures are
         *          internally allocated based on the the number of frames
         *          (nFrames) and the number of streams for each frame
         *          (nStreams).The FileHeader must be set first prior to
         *          setting the Stream structures.
         *
         * @return  Status
         *              - SUCCESS               If internal structures are
         *                                      updated successfully
         *              - FILE_HEADER_ERROR     If the file header structure is
         *                                      invalid
         *              - FAILED                If stream index is invalid or
         *                                      the actual data/string sizes
         *                                      does not match those in the
         *                                      stream info
         *              - INVALID_OPERATION     If API is invoked with the FSF
         *                                      file utility in READ mode
         */
        virtual FsfStatus SetStream(const unsigned int frameIndex,
                                    const unsigned int streamIndex,
                                    Stream &stream);

        /**
         * @brief Get Streams.
         *
         * @details The referenced Stream structure is populated with the
         *          contents of the selected stream from the opened FSF file.
         *          The FileHeader must be retrieved first prior to retrieving
         *          streams.
         *
         * @return  Status
         *              - SUCCESS               If the stream is retrieved from
         *                                      file successfully
         *              - FILE_HEADER_ERROR     If the file header structure is
         *                                      invalid
         *              - FILE_NOT_OPEN         If tried to stream contents
         *                                      with the file not open
         *              - FAILED                If there are errors encountered
         *                                      while reading from file or the
         *                                      stream index is invalid
         *              - INVALID_OPERATION     If API is invoked with the FSF
         *                                      file utility in WRITE mode
         */
        virtual FsfStatus GetStream(const unsigned int frameIndex,
                                    const unsigned int streamIndex,
                                    Stream &stream);
};

} // namespace aditof

#endif // FSF_H
