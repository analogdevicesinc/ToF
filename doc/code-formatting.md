# Code formatting

## Overview
All code within this repository needs to have the same formatting. 3rd party software imported into this repository are excluded from this rule.
The **ClangFormat** is used to format the code. The specifics on how the code should be formatted are written in file [.clang-format](/.clang-format) located in the top-level directory of the repository.

Contributors will have to format the code. The continuous integration (CI) process also checks whether the code is formatted or not.

The current version that is being used is ClangFormat 6.0.

## Setup

### Windows
The ClangFormat is part of the LLVM project. The 64-bit windows installer can be downloaded from: https://releases.llvm.org/6.0.0/LLVM-6.0.0-win64.exe.

NOTE: The above installer and installers for other configurations or platforms can be found at: section **Download LLVM 6.0.0** from page https://releases.llvm.org/download.html
