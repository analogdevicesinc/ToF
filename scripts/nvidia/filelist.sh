# 📦 Define your pattern list: "relative_src_pattern,relative_dst_folder"
COPY_PATTERNS=(
    # Python Examples
    "cp,$SRC_PREFIX/bindings/python/examples/first_frame/*.py,$EVALUATION/python",
    "cp,$SRC_PREFIX/bindings/python/examples/data_collect/*.py,$EVALUATION/python",
    "cp,$SRC_PREFIX/bindings/python/examples/streaming/*.py,$EVALUATION/python",
    "cp,$SRC_PREFIX/bindings/python/examples/dual_cameras/*.py,$EVALUATION/python",
    "cp,$SRC_PREFIX/bindings/python/examples/saveCCBToFile/*.py,$EVALUATION/python",
    # C++ Examples
    "cp,$SRC_PREFIX/examples/first-frame/first-frame,$EVALUATION/C++",
    "cp,$SRC_PREFIX/examples/data_collect/data_collect,$EVALUATION/C++",
    "cp,$SRC_PREFIX/examples/tof-viewer/ADIToFGUI,$EVALUATION/C++",
    "cp,$SRC_PREFIX/examples/tof-viewer/tof-tools.config,$EVALUATION/C++",
    # Tools
    ## ctrl_app
    "cp,$SRC_PREFIX/tools/debug_apps/ctrl_app/ctrl_app,$TOOLS/ctrl_app",
    "cp,$SRC_PREFIX/tools/debug_apps/ctrl_app/infile.txt,$TOOLS/ctrl_app",
    "cp,$SRC_PREFIX/tools/debug_apps/ctrl_app/readme.md,$TOOLS/ctrl_app",
    ## rawparser
    "cp,$SRC_PREFIX/../tools/misc/rawparser.py,$TOOLS/rawparser",
    "cp,$SRC_PREFIX/../tools/misc/README.md,$TOOLS/rawparser",
    ## V4L2 Scripts
    ### TODO
    ## NVM Tools
    ### TODO
    # Stage Library files
    "cp,$SRC_PREFIX/bindings/python/build/lib/aditofpython.cpython-310-aarch64-linux-gnu.so,$EVALUATION/python",
    "cp,/opt/ADI-ADCAM/*,$LIBS"
    "cp,$LIBS/lib/libaditof.*,$EVALUATION/python"
    "mv,$LIBS/lib/libaditof.*,$EVALUATION/C++"
)