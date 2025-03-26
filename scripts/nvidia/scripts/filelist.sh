# 📦 Define your pattern list: "relative_src_pattern,relative_dst_folder"
COPY_PATTERNS=(
    "$SRC_PREFIX/bindings/python/build/lib/aditofpython.cpython-310-aarch64-linux-gnu.so,$DST_PREFIX/bindings/python",
    "$SRC_PREFIX/bindings/python/examples/first_frame/*.py,$DST_PREFIX/bindings/python",
    "/opt/ADI/*,$BASE_DST_PREFIX/libs"
)