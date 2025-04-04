# 📦 Define your pattern list: "relative_src_pattern,relative_dst_folder"
COPY_PATTERNS=(
    # Python Examples
    "cp,$SRC_PREFIX/bindings/python/examples/first_frame/*.py,$EVALUATION/Python",
    "cp,$SRC_PREFIX/bindings/python/examples/data_collect/*.py,$EVALUATION/Python",
    "cp,$SRC_PREFIX/bindings/python/examples/streaming/*.py,$EVALUATION/Python",
    "cp,$SRC_PREFIX/bindings/python/examples/dual_cameras/*.py,$EVALUATION/Python",
    "cp,$SRC_PREFIX/bindings/python/examples/saveCCBToFile/*.py,$EVALUATION/Python",
    # C++ Examples
    ## first-frame
    "cp,$SRC_PREFIX/examples/first-frame/first-frame,$EVALUATION/C++/first-frame",
    "cp,$SRC_PREFIX/../examples/first-frame/main.cpp,$EVALUATION/C++/first-frame",
    "cp,$SRC_PREFIX/../examples/first-frame/Makefile.eval,$EVALUATION/C++/first-frame",
    ## data-collect
    "cp,$SRC_PREFIX/examples/data_collect/data_collect,$EVALUATION/C++/data_collect",
    "cp,$SRC_PREFIX/../examples/data_collect/main.cpp,$EVALUATION/C++/data_collect",
    "cp,$SRC_PREFIX/../examples/data_collect/Makefile.eval,$EVALUATION/C++/data_collect",
    ## common codes
    "cp,$SRC_PREFIX/../dependencies/adi/command_parser/command_parser.*,$EVALUATION/C++/common",
    ## ADIToFGUI
    "cp,$SRC_PREFIX/examples/tof-viewer/ADIToFGUI,$EVALUATION/C++/ADIToFGUI",
    "cp,$SRC_PREFIX/examples/tof-viewer/tof-tools.config,$EVALUATION/C++/ADIToFGUI",

    # Tools
    ## ctrl_app
    "cp,$SRC_PREFIX/tools/debug_apps/ctrl_app/ctrl_app,$TOOLS/ctrl_app",
    "cp,$SRC_PREFIX/tools/debug_apps/ctrl_app/infile.txt,$TOOLS/ctrl_app",
    "cp,$SRC_PREFIX/tools/debug_apps/ctrl_app/readme.md,$TOOLS/ctrl_app",
    ## rawparser: This is not in the build folder
    "cp,$SRC_PREFIX/../tools/misc/rawparser.py,$TOOLS/rawparser",
    "cp,$SRC_PREFIX/../tools/misc/README.md,$TOOLS/rawparser",
    ## aditof-server
    "cp,$SRC_PREFIX/apps/server/aditof-server,$TOOLS/aditof-server",
    ## V4L2 Scripts: This is not in the build folder
    "cp,$SRC_PREFIX/../tools/v4l2_scripts/ADCAM/*,$TOOLS/v4l2_scripts",
        #TODO: Missing some scripts
    ## NVM Tools
    "cp,$SRC_PREFIX/tools/nvm_tools/NVM_READ/NVM_READ,$TOOLS/nvm_tools",
    ## CCB Tools
    "cp,$SRC_PREFIX/tools/nvm_tools/CCB_READ/CCB_READ,$TOOLS/nvm_tools",
    # Stage Library files
    "cp,$SRC_PREFIX/bindings/python/build/lib/aditofpython.cpython-310-aarch64-linux-gnu.so,$EVALUATION/Python",
    "cp,$LIB_INSTALL_FOLDER/*,$LIBS/"
)