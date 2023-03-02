# Code formatting

## Overview
All code within this repository needs to have the same formatting. 3rd party software imported into this repository are excluded from this rule.
The **ClangFormat** is used to format the code. The specifics on how the code should be formatted are written in file [.clang-format](/.clang-format) located in the top-level directory of the repository.

Contributors will have to format the code. The continuous integration (CI) process also checks whether the code is properly formatted and will fail otherwise.

The current version that is being used is ClangFormat 14.0.

## Setup

### Windows
The **ClangFormat** is part of the LLVM project. Therefore, the LLVM installer needs to be installed on Windows. The 64-bit windows installer can be downloaded from: https://github.com/llvm/llvm-project/releases/download/llvmorg-14.0.6/LLVM-14.0.6-win64.exe.

NOTE: The above installer and installers for other configurations or platforms can be found at: section **Download LLVM 14.0.6** from page https://releases.llvm.org/download.html

#### Manually formatting file by file
```
cd C:\Users\a-user-name\repos\ToF
```
###### Command Prompt
```
"C:\Program Files\LLVM\bin\clang-format.exe" -i .\sdk\include\aditof\camera.h
```
##### PowerShell
```
&'C:\Program Files\LLVM\bin\clang-format.exe' -i .\sdk\include\aditof\camera.h
```

##### Automatically formatting only the modified files. Using ClangFormat with Visual Code
Open Visual Code. Click on the 'Extentions' icon from the left side bar or press simultaneoulsy Ctrl+Shift+x.
Locate the find (on the left part of the screen) and type clang-format. Pick the 'Clang-Format' from Xaver Hellauer. Then click 'Install'.
This extension should be able to detect the path where ClangFormat has been installed using the LLVM installer mentioned previously.
There are 2 options to format code:
1. Select code, right-click and 'Format Document' or use keyboard shortcut: Shift+Alt+F
2. Enable 'Format On Save' from settings. File->Preferences->Settings then Text Editor->Formatting


### Linux (Ubuntu 22.04)
The **ClanfFormat** cand be install via 'apt'. Open up a terminal and type:
```
sudo apt install clang-format
```
