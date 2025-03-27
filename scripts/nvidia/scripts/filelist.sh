# 📦 Define your pattern list: "relative_src_pattern,relative_dst_folder"
COPY_PATTERNS=(
    "$SRC_PREFIX/bindings/python/build/lib/aditofpython.cpython-310-aarch64-linux-gnu.so,$LIB_INSTALL_FOLDER/lib",
    # Python Examples
    "$SRC_PREFIX/bindings/python/examples/first_frame/*.py,$DST_PREFIX/python",
    "$SRC_PREFIX/bindings/python/examples/data_collect/*.py,$DST_PREFIX/python",
    "$SRC_PREFIX/bindings/python/examples/streaming/*.py,$DST_PREFIX/python",
    "$SRC_PREFIX/bindings/python/examples/dual_cameras/*.py,$DST_PREFIX/python",
    "$SRC_PREFIX/bindings/python/examples/saveCCBToFile/*.py,$DST_PREFIX/python",
    # C++ Examples
    "$SRC_PREFIX/examples/first-frame/first-frame,$DST_PREFIX/C++",
    "$SRC_PREFIX/examples/data_collect/data_collect,$DST_PREFIX/C++",
    # Stage Library files
    "/opt/ADI-ADCAM/*,$BASE_DST_PREFIX/libs"
)