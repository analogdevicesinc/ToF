## Steps To Download and Run The Project

* First Git Clone the project into a folder:
```
git clone https://bitbucket.analog.com/scm/tofi/adi_tof_gui.git
```
* After it finishes downloading the projec folders, then go to the "adi_tof_gui" folder and execute a submodule pdate:
```
cd adi_tof_gui
git submodule update --init --recursive
```
* By now, all submodules have been already downloaded and we have all the files ready to build. Let's make sure e are compiling from master branch to the following submodules:

```
<*PATH*>\adi_tof_gui>git checkout master
<*PATH*>\adi_tof_gui>cd external\aditof_cmos_sdk\src
<*PATH*>\adi_tof_gui\external\aditof_cmos_sdk\src>git checkout master
<*PATH*>\adi_tof_gui\external\aditof_cmos_sdk\src>cd ../../
<*PATH*>\adi_tof_gui\external>cd FSF\src
<*PATH*>\adi_tof_gui\external\FSF\src>git checkout master
```
* Next go back to "adi_tof_gui" folder and let's build the code
```
<*PATH*>\adi_tof_gui>mkdir build
<*PATH*>\adi_tof_gui>cd build
<*PATH*>\adi_tof_gui\build>cmake -G "Visual Studio 16 2019" -A x64 ..
```
* Once the project solution has been created, it's time to compile either Debug or Release
(For Release type)
```
cmake --build . --target ALL_BUILD --config Release
```
* One more step. This step is necessary for now. Manually copy and paste the following files.
   * Copy from <*PATH*>\adi_tof_gui\external\aditof_cmos_sdk\src\sdk\lib\windows
     * tofi_compute.dll
        * tofi_compute.lib
        * tofi_config.dll
        * tofi_config.lib
        * tofi_processor.obj
   * Paste to <*PATH*>\adi_tof_gui\build\Release AND <*PATH*>\adi_tof_gui\build\Debug folders

* Finally we need to Copy and Paste the calibration files depending of the module used. For example, for our oro module
   * Copy from <*PATH*>\adi_tof_gui\external\aditof_cmos_sdk\src\sdk\config
     * camera_calibration.ccb
        * camera_configuration_toro.cfg
        * CCB_1909-0180039.ccb
        * config_toro.json
        * RawToDepth
   * Paste to <*PATH*>\adi_tof_gui\build and <*PATH*>\adi_tof_gui\build\Release folders

By now we should have an executable in either Debug or Release folder named "ADIToFGUI.exe". Doulble click on it and the graphical user interface will run.
