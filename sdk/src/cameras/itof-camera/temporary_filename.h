// Portions Copyright (c) 2020 Analog Devices, Inc.
//
// Copyright 2008, Google Inc.
// All rights reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
//     * Neither the name of Google Inc. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <cstdint>
#include <string>
#include <cstdio>
#include <glog/logging.h>

#if _WIN32
# include <windows.h>
# include <io.h>
#include <fcntl.h>
#else
# include <unistd.h>
#endif  // _WIN32

std::string getTempFilename(std::string prefix) {
    int fhandle = -1;
#ifdef _WIN32
    char temp_dir_path[MAX_PATH + 1] = { '\0' };  // NOLINT
    char temp_file_path[MAX_PATH + 1] = { '\0' };  // NOLINT
    ::GetTempPathA(sizeof(temp_dir_path), temp_dir_path);
    const UINT success = ::GetTempFileNameA(temp_dir_path,
                                            prefix.c_str(),
                                            0,  // Generate unique file name.
                                            temp_file_path);
    if(success == 0) {
      LOG(ERROR) << "Unable to create a temporary file in " << temp_dir_path;
    }
    errno_t  err = _sopen_s(&fhandle,temp_file_path,_O_BINARY,_SH_DENYNO,_S_IREAD|_S_IWRITE);
    if(err == 0) {
      _close( fhandle );
    }
# else
    const int max_path = 4096;
    char temp_file_path[max_path] = {0};
    if (prefix.length()>=512){
      LOG(ERROR) << "Unable to create a temporary file with prefix " << prefix;
      return "";
    }

    // There's no guarantee that a test has write access to the current
    // directory, so we create the temporary file in the /tmp directory
    // instead. We use /tmp on most systems, and /sdcard on Android.
    // That's because Android doesn't have /tmp.
#  if OS_LINUX_ANDROID
    // Note: Android applications are expected to call the framework's
    // Context.getExternalStorageDirectory() method through JNI to get
    // the location of the world-writable SD Card directory. However,
    // this requires a Context handle, which cannot be retrieved
    // globally from native code. Doing so also precludes running the
    // code as part of a regular standalone executable, which doesn't
    // run in a Dalvik process (e.g. when running it through 'adb shell').
    //
    // The location /sdcard is directly accessible from native code
    // and is the only location (unofficially) supported by the Android
    // team. It's generally a symlink to the real SD Card mount point
    // which can be /mnt/sdcard, /mnt/sdcard0, /system/media/sdcard, or
    // other OEM-customized locations. Never rely on these, and always
    // use /sdcard.
    strcat(temp_file_path, "/sdcard/");
#  else
    strcat(temp_file_path, "/tmp/");
    //char temp_file_path[] = "/tmp/adi_tofcam_.XXXXXX";
#  endif  // OS_LINUX_ANDROID
    strcat(temp_file_path, prefix.c_str());
    strcat(temp_file_path, ".XXXXXX");
    fhandle = mkstemp(temp_file_path);
    if(fhandle == -1) {
        close(fhandle);
    }
# endif

    std::string filename = "";
    if(fhandle == -1) {
        LOG(ERROR) << "Unable to open temporary file " << temp_file_path;
    }
    else  {
        filename = temp_file_path;
    }

    return filename;
}

