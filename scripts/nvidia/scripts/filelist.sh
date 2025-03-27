# 📦 Define your pattern list: "relative_src_pattern,relative_dst_folder"
COPY_PATTERNS=(
    # Python Examples
    "$SRC_PREFIX/bindings/python/examples/first_frame/*.py,$DST_PREFIX/python",
    "$SRC_PREFIX/bindings/python/examples/data_collect/*.py,$DST_PREFIX/python",
    "$SRC_PREFIX/bindings/python/examples/streaming/*.py,$DST_PREFIX/python",
    "$SRC_PREFIX/bindings/python/examples/dual_cameras/*.py,$DST_PREFIX/python",
    "$SRC_PREFIX/bindings/python/examples/saveCCBToFile/*.py,$DST_PREFIX/python",
    # C++ Examples
    "$SRC_PREFIX/examples/first-frame/first-frame,$DST_PREFIX/C++",
    "$SRC_PREFIX/examples/data_collect/data_collect,$DST_PREFIX/C++",
    "$SRC_PREFIX/examples/tof-viewer/ADIToFGUI,$DST_PREFIX/C++",
    "$SRC_PREFIX/examples/tof-viewer/tof-tools.config,$DST_PREFIX/C++",
    # Tools
    ## ctrl_app
    "$SRC_PREFIX/tools/debug_apps/ctrl_app/ctrl_app,$DST_PREFIX/tools/ctrl_app",
    "$SRC_PREFIX/tools/debug_apps/ctrl_app/infile.txt,$DST_PREFIX/tools/ctrl_app",
    "$SRC_PREFIX/tools/debug_apps/ctrl_app/readme.md,$DST_PREFIX/tools/ctrl_app",
    ## rawparser
    "$SRC_PREFIX/../tools/misc/rawparser.py,$DST_PREFIX/tools/rawparser",
    "$SRC_PREFIX/../tools/misc/README.md,$DST_PREFIX/tools/rawparser",
    ## V4L2 Scripts
    ### TODO
    ## NVM Tools
    ### TODO
    # Stage Library files
    "$SRC_PREFIX/bindings/python/build/lib/aditofpython.cpython-310-aarch64-linux-gnu.so,$DST_PREFIX/python",
    "/opt/ADI-ADCAM/*,$BASE_DST_PREFIX/libs"
)