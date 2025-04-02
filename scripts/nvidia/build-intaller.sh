#!/bin/bash

DESTINATION="./output"
RESOURCES="./adcam-installer/resources"
SRC_PREFIX="../../build"
EVALUATION="$DESTINATION/eval"
TOOLS="$DESTINATION/tools"
DEVELOPMENT="$DESTINATION/dev"
KERNEL="$DESTINATION/kernel"
LIBS="$DESTINATION/libs"
LIB_INSTALL_FOLDER="/opt/ADI-ADCAM"
GIT_CLONE_SCRIPT_NAME="$DEVELOPMENT/git_clone_tof.sh"
RUNME_SCRIPT_NAME="$DESTINATION/run_me.sh"
COPY_LOG="./copied_files.txt"
CONFIG_JSON="./config.json"
BRANCH=$(git rev-parse --abbrev-ref HEAD)
JETPACK_VERSION="Unknown"
VERSION="Unknown"
FORCE=false
UPDATE=false
BUILDALL=false
COMPRESS=false

# Parse arguments
print_help() {
    echo "🔧 Usage: $0 [-f] [-h][-u][-a][-c]"
    echo ""
    echo "  -f    Force rebuild by deleting the existing build directory"
    echo "  -u    Update the Ubuntu dependencies"
    echo "  -a    Build installation package to include SDK and Kernel components" 
    echo "  -c    Compress Rust generated binary."
    echo "  -h    Show this help message"
}

# Parse options
while getopts ":fhuac" opt; do
    case $opt in
        f)
            FORCE=true
            ;;
        u)
            UPDATE=true
            ;;
        a)
            BUILDALL=true
            ;;
        c)
            COMPRESS=true
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

clean_folder_contents() {
    local target_dir="$1"

    if [ -z "$target_dir" ]; then
        echo "❌ Usage: clean_folder_contents <target_dir>"
        return 1
    fi

    if [ ! -d "$target_dir" ]; then
        echo "📁 Creating directory: $target_dir"
        mkdir -p "$target_dir"
    fi

    if [ -d "$target_dir" ]; then
        echo "🧹 Cleaning all contents inside: $target_dir"
        find "$target_dir" -mindepth 1 -delete
        echo "✅ Folder cleaned!"
    else
        echo "❌ Failed to access directory: $target_dir"
        return 1
    fi

    rmdir "$target_dir"
}

#########################
# Extract the versions
#########################
get_sdk_version() {

    CMAKE_FILE="../../CMakeLists.txt"

    if [ ! -f "$CMAKE_FILE" ]; then
        echo "Cannot find the CMakeLists.txt file."
        exit 1
    fi

    # Extract version components
    MAJOR=$(grep -oP 'set\s*\(\s*ADITOF_VERSION_MAJOR\s+\K[0-9]+' "$CMAKE_FILE")
    MINOR=$(grep -oP 'set\s*\(\s*ADITOF_VERSION_MINOR\s+\K[0-9]+' "$CMAKE_FILE")
    PATCH=$(grep -oP 'set\s*\(\s*ADITOF_VERSION_PATCH\s+\K[0-9]+' "$CMAKE_FILE")
    # Combine into full version
    VERSION="$MAJOR.$MINOR.$PATCH"

    echo "📦 Extracted ADITOF version: $VERSION"
}

get_jetpack_version() {

    JETPACK_FILE="/etc/nv_tegra_release"
    JETPACK_VERSION="unknown"

    if [ -f "$JETPACK_FILE" ]; then
        # Extract the L4T major version (e.g., R36)
        L4T_MAJOR=$(grep -oP 'R\d+' "$JETPACK_FILE" | head -n 1 | tr -d 'R')

        # Extract the L4T minor version (e.g., 4.3)
        L4T_REVISION=$(grep "REVISION:" "$JETPACK_FILE" | sed -n 's/.*REVISION: \([^,]*\).*/\1/p')

        FULL_L4T="${L4T_MAJOR}.${L4T_REVISION}"

        echo "$FULL_L4T"
        case "$FULL_L4T" in
            36.4.3) JETPACK_VERSION="JP62" ;;
            *)      
                    echo "Unsupported JetPack version, terminating script."
                    exit 1
                    ;;
        esac

        echo "🔍 Detected JetPack version: $JETPACK_VERSION"
    else
        echo "❌ $JETPACK_FILE not found. Are you on a Jetson device?"
        exit 1
    fi

    if [ ! -f "$CONFIG_JSON" ]; then
        echo "❌ JSON file not found: $CONFIG_JSON"
        exit 1
    fi

    # Use jq to add or update "jetpack"
    jq --arg ver "$JETPACK_VERSION" '.jetpack = $ver' "$CONFIG_JSON" > tmp.json && mv tmp.json "$CONFIG_JSON"

    echo "✅ jetpack version set to: $JETPACK_VERSION in $JSON_FILE"

    return $JETPACK_VERSION
}

update_dependencies() {
    # List of required packages
    if [ "$UPDATE" = true ]; then
        # Update package list first
        echo "Updating package list..."
        sudo apt update
    fi

    REQUIRED_PACKAGES=(
        upx
        jq
        cmake
        g++
        gcc
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
}

build_eval_kit() {

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
        -S ../../ \
        -B "$SRC_PREFIX"
    cmake --build "$SRC_PREFIX"
    sudo cmake --install "$SRC_PREFIX"
    if [ -d "./CMakeFiles" ]; then
        echo "Cleaning up CMake-generated files in: $SRC_PREFIX"

        rm -rf "./CMakeFiles"
    fi 

    # Check if Rust is installed
    if command -v rustc >/dev/null 2>&1; then
        echo "✅ Rust is already installed (version $(rustc --version))"
    else
        echo "🛠 Installing Rust for ARM64..."

        # Download and run rustup-init
        curl https://sh.rustup.rs -sSf | sh -s -- -y --default-toolchain stable

        # Add Cargo to PATH for current shell session
        export PATH="$HOME/.cargo/bin:$PATH"

        echo "✅ Rust installed (version $(rustc --version))"
    fi
}

#########################
### Setup staging folder
#########################
setup_staging() {

    # Clean staging folder
    clean_folder_contents "$EVALUATION"

    # Created needed folders
    mkdir -p "$EVALUATION"
    mkdir -p "$TOOLS"
    mkdir -p "$DEVELOPMENT"
    mkdir -p "$KERNEL"

    # ✨ Clear log file
    > "$COPY_LOG"

    source ./filelist.sh

    
    for entry in "${COPY_PATTERNS[@]}"; do
        IFS=',' read -r OP REL_SRC_PATTERN REL_DST_DIR <<< "$entry"

        OPERATION="$OP"
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
                USE_SUDO=""
                if [ "$USE_SUDO" = true ]; then
                    USE_SUDO="sudo "
                fi

                OP_FAILED=true
                if [ "$OPERATION" = "mv" ]; then
                    "$USE_SUDO"mv "$file" "$DST_DIR/"
                    OP_FAILED=false
                else
                    "$USE_SUDO"cp -r "$file" "$DST_DIR/"
                    OP_FAILED=false
                fi

                if [ "$OP_FAILED" = true ]; then
                    echo "❌ Operation failed: $entry"
                    exit
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

    sudo chown -R astraker:astraker "$DESTINATION"/*
    sudo rm -rf "$LIB_INSTALL_FOLDER"


    echo "✅ Done copying files!"
    echo "📄 File list saved to: $COPY_LOG"
}

#########################
# Build the kernel components
#########################
build_kernel() {
    DRIVER_FOLDER=../../sdcard-images-utils/nvidia
    if [ "$BUILDALL" = true ]; then
        "$DRIVER_FOLDER"/runme.sh "$VERSION" "$BRANCH"
    fi

    MATCH=$(compgen -G "${DRIVER_FOLDER}/NVIDIA_ToF_ADSD3500_REL_PATCH_*.zip" | head -n 1)

    if [ -n "$MATCH" ]; then
        echo "✅ Building with kernel bits: $MATCH"
        cp "$MATCH" "$KERNEL"
    else
        echo "⚠️ No matching .zip file found - Building without the Kernel bits."
        pause 10
    fi
}

#########################
# Create Git clone script in the staging folder for the current branch
#########################
build_git_clone_sh() {
    # Write the embedded script to disk
    #cat << EOF > "$GIT_CLONE_SCRIPT_NAME"
    ##!/bin/bash

    #set -e

    #DEFAULT_VERSION="$BRANCH"
    #VERSION="\${1:-\$DEFAULT_VERSION}"

    #echo "Cloning branch \$VERSION from ToF repo..."
    #git clone --branch "\$VERSION" https://github.com/analogdevicesinc/ToF ToF-"\$VERSION"
    #cd ToF-"\$VERSION"
    #git submodule update --init
    #git checkout "\$VERSION"
    #cd libaditof
    #git checkout "\$VERSION"
    #cd ..

    #echo "✅ Done!"
    #EOF

    echo "#!/bin/bash" > "$GIT_CLONE_SCRIPT_NAME"
    echo '' >> "$GIT_CLONE_SCRIPT_NAME"
    echo set -e >> "$GIT_CLONE_SCRIPT_NAME"
    echo '' >> "$GIT_CLONE_SCRIPT_NAME"
    echo DEFAULT_VERSION="$BRANCH" >> "$GIT_CLONE_SCRIPT_NAME"
    echo VERSION=\"\${1:-\$DEFAULT_VERSION}\" >> "$GIT_CLONE_SCRIPT_NAME"
    echo '' >> "$GIT_CLONE_SCRIPT_NAME"
    echo echo "Cloning branch $VERSION from ToF repo..." >> "$GIT_CLONE_SCRIPT_NAME"
    echo git clone https://github.com/analogdevicesinc/ToF ToF-\"\$VERSION\" >> "$GIT_CLONE_SCRIPT_NAME"
    echo cd ToF-\"\$VERSION\" >> "$GIT_CLONE_SCRIPT_NAME"
    echo git submodule update --init >> "$GIT_CLONE_SCRIPT_NAME"
    echo git checkout \"\$VERSION\" >> "$GIT_CLONE_SCRIPT_NAME"
    echo cd libaditof >> "$GIT_CLONE_SCRIPT_NAME"
    echo git checkout \"\$VERSION\" >> "$GIT_CLONE_SCRIPT_NAME"
    echo cd .. >> "$GIT_CLONE_SCRIPT_NAME"
    echo '' >> "$GIT_CLONE_SCRIPT_NAME"
    echo echo "✅ Done!" >> "$GIT_CLONE_SCRIPT_NAME"


    # Make it executable
    chmod +x "$GIT_CLONE_SCRIPT_NAME"

    echo "$GIT_CLONE_SCRIPT_NAME" >> "$COPY_LOG"

    echo "Created script: $GIT_CLONE_SCRIPT_NAME"
}

#########################
# Create script to point to library files
#########################
build_run_me_sh() {
    #cat << EOF > "$RUNME_SCRIPT_NAME"
    ##!/bin/bash

    #LIBS_PATH="\$(realpath "libs")"
    #echo "Creating Directory Path Soft Link: \$LIBS_PATH"
    #sudo ln -s "\$LIBS_PATH" "$LIB_INSTALL_FOLDER"

    #echo "\$LIBS_PATH/lib" | sudo tee /etc/ld.so.conf.d/adi-adcam.conf
    #sudo ldconfig

    #EOF

    echo '#!/bin/bash' > "$RUNME_SCRIPT_NAME"
    echo '' >> "$RUNME_SCRIPT_NAME"
    echo LIBS_PATH="$(realpath "libs")" >> "$RUNME_SCRIPT_NAME"
    echo echo "Creating Directory Path Soft Link: $LIBS_PATH" >> "$RUNME_SCRIPT_NAME"
    echo sudo ln -s "$LIBS_PATH" "$LIB_INSTALL_FOLDER" >> "$RUNME_SCRIPT_NAME"
    echo '' >> "$RUNME_SCRIPT_NAME"
    echo echo "$LIBS_PATH/lib" | sudo tee /etc/ld.so.conf.d/adi-adcam.conf >> "$RUNME_SCRIPT_NAME"
    echo sudo ldconfig >> "$RUNME_SCRIPT_NAME"

    if [ ! -f "$RUNME_SCRIPT_NAME" ]; then
        echo "❌ Stage file not created: $RUNME_SCRIPT_NAME"
        return -1
    fi

    chmod +x "$RUNME_SCRIPT_NAME"

    echo "$RUNME_SCRIPT_NAME" >> "$COPY_LOG"

    echo "Created script: $RUNME_SCRIPT_NAME"
}

#########################
# Create permissions.json
#########################
create_permissions_json() {
    set -e

    PERMISSIONS_FILE="$RESOURCES/permissions.json"

    # Ensure jq is installed
    if ! command -v jq &>/dev/null; then
        echo "❌ jq is required but not installed."
        exit 1
    fi

    # Initialize empty JSON object
    echo "{}" > "$PERMISSIONS_FILE"

    # Walk through all files under the directory
    find "$DESTINATION" -type f | while read -r file; do
        # Strip the DESTINATION prefix
        rel_path="${file#$DESTINATION/}"

        # Get file permissions (mode) in octal
        mode=$(stat -c "%a" "$file")

        # Add entry to JSON using jq
        tmp=$(mktemp)
        jq --arg key "$rel_path" --arg value "$mode" '. + {($key): ($value | tonumber)}' "$PERMISSIONS_FILE" > "$tmp" && mv "$tmp" "$PERMISSIONS_FILE"
    done

    echo "✅ Permissions written to: $PERMISSIONS_FILE"
}

#########################
# Build the installer
#########################
build_installer() {
    # Create output.zip
    STAGING_FILE=$(realpath install-bundle.tgz)
    tar -czf "$STAGING_FILE" -C "$DESTINATION/" .

    if [ ! -f "$STAGING_FILE" ]; then
        echo "❌ Stage file not created: $STAGING_FILE"
        return -1
    fi

    # Build final package
    echo "Starting build of installer binary: "
    make -C ./adcam-installer clean
    make_output=$(make -C ./adcam-installer package JETPACKVERSION="$JETPACK_VERSION" BUNDLE="$STAGING_FILE")
    installer_path=$(echo "$make_output" | grep "^BuiltXYZ: " | awk '{print $2}')
    installer_path=./adcam-installer/"$installer_path"
    if [ "$COMPRESS" = true ]; then
        echo "✅ Compressing $installer_path"
        upx --best --lzma "$installer_path"
    else
        echo "⚠️ Not compressing $installer_path"
    fi

    return 0
}

#########################
# Final clean up
#########################
clean_up() {
    echo "✅ Cleaning up on exit!"
    clean_folder_contents "$DESTINATION"
    rm "$STAGING_FILE"
    rm "$COPY_LOG"
}

main() {
    get_sdk_version
    get_jetpack_version
    update_dependencies
    build_eval_kit
    setup_staging
    build_kernel
    build_git_clone_sh
    build_run_me_sh
    create_permissions_json
    build_installer
}

trap clean_up EXIT
main