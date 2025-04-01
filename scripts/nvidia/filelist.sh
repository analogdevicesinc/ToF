# 📦 Define your pattern list: "relative_src_pattern,relative_dst_folder"
COPY_PATTERNS=(
    # Python Examples
    "$SRC_PREFIX/bindings/python/examples/first_frame/*.py,$EVALUATION/python",
    "$SRC_PREFIX/bindings/python/examples/data_collect/*.py,$EVALUATION/python",
    "$SRC_PREFIX/bindings/python/examples/streaming/*.py,$EVALUATION/python",
    "$SRC_PREFIX/bindings/python/examples/dual_cameras/*.py,$EVALUATION/python",
    "$SRC_PREFIX/bindings/python/examples/saveCCBToFile/*.py,$EVALUATION/python",
    # C++ Examples
    "$SRC_PREFIX/examples/first-frame/first-frame,$EVALUATION/C++",
    "$SRC_PREFIX/examples/data_collect/data_collect,$EVALUATION/C++",
    "$SRC_PREFIX/examples/tof-viewer/ADIToFGUI,$EVALUATION/C++",
    "$SRC_PREFIX/examples/tof-viewer/tof-tools.config,$EVALUATION/C++",
    # Tools
    ## ctrl_app
    "$SRC_PREFIX/tools/debug_apps/ctrl_app/ctrl_app,$TOOLS/ctrl_app",
    "$SRC_PREFIX/tools/debug_apps/ctrl_app/infile.txt,$TOOLS/ctrl_app",
    "$SRC_PREFIX/tools/debug_apps/ctrl_app/readme.md,$TOOLS/ctrl_app",
    ## rawparser
    "$SRC_PREFIX/../tools/misc/rawparser.py,$TOOLS/rawparser",
    "$SRC_PREFIX/../tools/misc/README.md,$TOOLS/rawparser",
    ## V4L2 Scripts
    ### TODO
    ## NVM Tools
    ### TODO
    # Stage Library files
    "$SRC_PREFIX/bindings/python/build/lib/aditofpython.cpython-310-aarch64-linux-gnu.so,$EVALUATION/python",
    "/opt/ADI-ADCAM/*,$LIBS"
)