@ECHO off
SETLOCAL

::this script downloads and installs the dependencies (glog, protobuf and websockets) and builds the sdk.

::global variables
for %%F in (cd %0 ..) do set source_dir=%%~dpF
set /a display_help=0
set /a answer_yes=0
set /a threads=4


set generator=""
set tof_dire=%CD%..\..\
set build_dire=..\..\build

set config_type=""
set generator=""
set /a set_config=0
set /a set_generator=0

::interpret the arguments
:interpret_arg
if "%1" neq "" (
   if /I "%1" EQU "-h" (
   set /a display_help=1
   shift
   goto :interpret_arg
   )
   if /I "%1" EQU "--help" (
   set /a display_help=1
   shift
   goto :interpret_arg
   )
   if /I "%1" EQU "-y" (
   set /a answer_yes=1
   shift
   goto :interpret_arg
   )
   if /I "%1" EQU "--yes" (
   set /a answer_yes=1
   shift
   goto :interpret_arg
   )
   if /I "%1" EQU "-g" (
   set generator=%2
   set /a set_generator=1
   shift
   shift
   goto :interpret_arg
   )
   if /I "%1" EQU "--generator" (
   set generator=%2
   set /a set_generator=1
   shift
   shift
   goto :interpret_arg
   )
   if /I "%1" EQU "-c" (
   set config_type=%2
   set /a set_config=1
   shift
   shift
   goto :interpret_arg
   )
   if /I "%1" EQU "--configuration" (
   set config_type=%2
   set /a set_config=1
   shift
   shift
   goto :interpret_arg
   )
   if /I "%1" EQU "-j" (
   set threads=%2
   shift
   shift
   goto :interpret_arg
   )
   shift
   goto :interpret_arg
)

if %display_help%==1 (
   call :print_help
   EXIT /B 0
   )

::check if the configuration is correct
set /a opt=0
if "%config_type%"=="Release" (
    set /a opt=1
)
if "%config_type%"=="Debug" (
    set /a opt=1
)
if %set_config%==0 ( 
    set /a opt=1
    set config_type=Release
    )
if %opt%==0 (
    echo Please enter a correct configuration (Release or Debug^)
    EXIT /B %ERRORLEVEL%
)
echo Setup will continue with the configuration: %config_type%

::check if the generator is correct
set /a opt=0
set opencv_vs=15
if %generator%=="Visual Studio 17 2022" (
    set /a opt=1
    set opencv_vs=15
)
if %generator%=="Visual Studio 16 2019" (
    set /a opt=1
    set opencv_vs=15
)
if %generator%=="Visual Studio 15 2017 Win64" (
    set /a opt=1
    set opencv_vs=15
)
if %generator%=="Visual Studio 14 2015 Win64" (
    set /a opt=1
    set opencv_vs=14
)

if %set_generator%==0 (
   set /a opt=1
   set generator="Visual Studio 17 2022"
   set opencv_vs=16
   )
if %opt%==0 (
    echo Please enter a correct configuration ("Visual Studio 17 2022";"Visual Studio 16 2019"; "Visual Studio 15 2017 Win64" or "Visual Studio 14 2015 Win64"^)
    EXIT /B %ERRORLEVEL%
)
echo Setup will continue with the generator: %generator%


echo The sdk will be built in: %build_dire%


::ask for permission to continue the setup
if %answer_yes%==0 (
   call :yes_or_exit "Do you want to continue?"
   )

::create the missing folders
if not exist %build_dire% md %build_dire%


::init and update of libaditof submodule
echo "Cloning sub modules"
pushd %tof_dire%
git submodule update --init --recursive
popd

::build the project with the selected options
pushd %build_dire%
cmake -G %generator% -DWITH_PYTHON=on -DRECV_ASYNC=off %source_dir% -DCMAKE_BUILD_TYPE=%config_type%
cmake --build . --config %config_type% -j %threads%
popd
EXIT /B %ERRORLEVEL%

:print_help
ECHO setup.bat [OPTIONS]
ECHO -h^|--help
ECHO        Print a usage message briefly summarizing the command line options available, then exit.
ECHO -y^|--yes
ECHO        Automatic yes to prompts.
ECHO -g^|--generator
ECHO        Visual Studio 17 2022 = Generates Visual Studio 2022 project files.
ECHO        Visual Studio 16 2019 = Generates Visual Studio 2019 project files.
ECHO        Visual Studio 15 2017 Win64 = Generates Visual Studio 2017 project files.
ECHO        Visual Studio 14 2015 Win64 = Generates Visual Studio 2015 project files.
ECHO -c^|--configuration
ECHO        Release = Configuration for Release build.
ECHO        Debug   = Configuration for Debug build.
ECHO -j
ECHO        Set the number of threads used for building, by default is set to 4.
EXIT /B 0

:yes_or_exit
:choice
set /P c="%~1 [Y/N]"
if /I "%c%" EQU "Y" goto :end_yes_or_exit
if /I "%c%" EQU "N" EXIT 
goto :choice
:end_yes_or_exit
EXIT /B 0

