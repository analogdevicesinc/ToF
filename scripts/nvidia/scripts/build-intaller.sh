#!/bin/bash
BASE_DST_PREFIX="../adcam-installer/resources"
SRC_PREFIX="../../../build"
DST_PREFIX="$BASE_DST_PREFIX/bin"
FORCE=false
UPDATE=false
LIB_INSTALL_FOLDER="/opt/ADI"
GIT_CLONE_SCRIPT_NAME="$BASE_DST_PREFIX/git_clone_tof.sh"
COPY_LOG="copied_files.txt"

# Parse arguments
print_help() {
    echo "🔧 Usage: $0 [-f] [-h][-u"
    echo ""
    echo "  -f    Force rebuild by deleting the existing build directory"
    echo "  -h    Show this help message"
}

# Parse options
while getopts ":fh" opt; do
    case $opt in
        f)
            FORCE=true
            ;;
        u)
            UPDATE=true
            ;;
        h)
            print_help
            exit 0
            ;;
        \?)
            echo "❌ Invalid option: -$OPTARG" >&2
            print_help
            exit 1
            ;;
    esac
done

#########################
# Extract the version
#########################

CMAKE_FILE="../../../CMakeLists.txt"

# Extract version components
MAJOR=$(grep -oP 'set\s*\(\s*ADITOF_VERSION_MAJOR\s+\K[0-9]+' "$CMAKE_FILE")
MINOR=$(grep -oP 'set\s*\(\s*ADITOF_VERSION_MINOR\s+\K[0-9]+' "$CMAKE_FILE")
PATCH=$(grep -oP 'set\s*\(\s*ADITOF_VERSION_PATCH\s+\K[0-9]+' "$CMAKE_FILE")
# Combine into full version
VERSION="$MAJOR.$MINOR.$PATCH"

echo "📦 Extracted ADITOF version: $VERSION"
#########################
# Build the device driver
#########################

# TODO

#########################
# Build the ToF Repo - bindings, example, sdk
#########################

# List of required packages
if [ "$UPDATE" = true ]; then
    # Update package list first
    echo "Updating package list..."
    sudo apt update
fi


REQUIRED_PACKAGES=(
    cmake
    g++
    libopencv-contrib-dev
    libopencv-dev
    libgl1-mesa-dev
    libglfw3-dev
    doxygen
    graphviz
    python3.10-dev
)
# Check and install each package
for pkg in "${REQUIRED_PACKAGES[@]}"; do
    if dpkg -s "$pkg" &> /dev/null; then
        echo "$pkg is already installed ✅"
    else
        echo "$pkg is missing, installing... ⏳"
        sudo apt install -y "$pkg"
    fi
done

# Only remove if the folder exists
if [ "$FORCE" = true ]; then
    echo "Removing existing build directory: $SRC_PREFIX"
    if [ -d "$SRC_PREFIX" ]; then
        rm -rf "$SRC_PREFIX"
    fi
else
    echo "No existing build directory to remove."
fi

# Run CMake to generate build system
echo "Generating build files in: $SRC_PREFIX"
cmake -DCMAKE_INSTALL_PREFIX="$LIB_INSTALL_FOLDER" \
      -DCMAKE_INSTALL_RPATH="$LIB_INSTALL_FOLDER"\lib \
      -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON \
      -DON_NVIDIA=ON \
      -DWITH_NETWORK=OFF \
      -S ../../../ \
      -B "$SRC_PREFIX"
cmake --build "$SRC_PREFIX"
sudo cmake --install "$SRC_PREFIX"
if [ -d "./CMakeFiles" ]; then
    echo "Cleaning up CMake-generated files in: $SRC_PREFIX"

    rm -rf "./CMakeFiles"
fi 

#########################
### Clean staging folder
#########################
if [ -d "$DST_PREFIX" ]; then
    echo "Cleaning all contents inside: $DST_PREFIX"

    # Deletes everything inside the folder — but not the folder itself
    find "$DST_PREFIX" -mindepth 1 -delete

    echo "🧼 Folder cleaned!"
else
    echo "❌ Directory does not exist: $DST_PREFIX"
fi

#########################
### Copy files to staging folder
#########################

# ✨ Clear log file
> "$COPY_LOG"

source ./filelist.sh

for entry in "${COPY_PATTERNS[@]}"; do
    IFS=',' read -r REL_SRC_PATTERN REL_DST_DIR <<< "$entry"

    SRC_PATTERN="$REL_SRC_PATTERN"
    DST_DIR="$REL_DST_DIR"

    echo "Processing: '$SRC_PATTERN' → '$DST_DIR'"

    # Create destination directory
    if [[ "$DST_DIR" == /opt/* || "$DST_DIR" == /usr/* ]]; then
        sudo mkdir -p "$DST_DIR"
        USE_SUDO=true
    else
        mkdir -p "$DST_DIR"
        USE_SUDO=false
    fi

    for file in $SRC_PATTERN; do
        if [ -e "$file" ]; then
            echo "  ↪ Copying $file → $DST_DIR"

            # Copy using appropriate permission
            if [ "$USE_SUDO" = true ]; then
                sudo cp -r "$file" "$DST_DIR/"
            else
                cp -r "$file" "$DST_DIR/"
            fi

            # Determine how to log files based on type
            if [ -d "$file" ]; then
                find "$file" -type f | while read -r subfile; do
                    rel_path="${subfile#$file/}"                    # relative to the source dir
                    dest_path="$DST_DIR/$(basename "$file")/$rel_path"
                    echo "$dest_path" >> "$COPY_LOG"
                done
            else
                dest_path="$DST_DIR/$(basename "$file")"
                echo "$dest_path" >> "$COPY_LOG"
            fi
        else
            echo "  ⚠️  No match for: $file"
        fi
    done
done


echo "✅ Done copying files!"
echo "📄 File list saved to: $COPY_LOG"

#########################
# Create Git clone script in the staging folder for the current branch
#########################
# Write the embedded script to disk
cat << 'EOF' > "$GIT_CLONE_SCRIPT_NAME"
#!/bin/bash

set -e

VERSION="\$1"

if [ -z "\$VERSION" ]; then
  echo "Usage: \$0 <version>"
  echo "Example: \$0 v5.0.0"
  exit 1
fi

echo "Cloning branch \$VERSION from ToF repo..."
git clone --branch "\$VERSION" https://github.com/analogdevicesinc/ToF
cd ToF
git submodule update --init

echo "✅ Done!"
EOF

# Make it executable
chmod +x "$GIT_CLONE_SCRIPT_NAME"

echo "$GIT_CLONE_SCRIPT_NAME" >> "$COPY_LOG"

echo "Created script: $GIT_CLONE_SCRIPT_NAME"