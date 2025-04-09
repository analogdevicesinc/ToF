#!/bin/bash

VERSION=$1
BRANCH=$2

pushd .
git clone https://github.com/analogdevicesinc/ToF "$BRANCH"
cd "$BRANCH"
git submodule update --init --recursive
git checkout "$BRANCH"
cd libaditof
git checkout "$BRANCH"
cd ../sdcard-images-utils/nvidia
sudo source ./setup.sh
source ./runme.sh "$VERSION" "$BRANCH"
# At the end the user is in the build folder.
cd ..

# Check if the output file has been created.
target_filename="NVIDIA_ToF_ADSD3500_REL_PATCH_$(date +%d%b%y).zip"
start_dir="."  # Replace with the directory to start searching

find "$start_dir" -type f -path "$start_dir/$target_filename" -print

if [ $? -eq 0 ]; then
    echo "File with full path '$start_dir/$target_filename' exists."
    mv "$(realpath $start_dir/$target_filename)" ../../../NVIDIA_ToF_ADSD3500_REL_PATCH.zip
    rm -rf ./build
    popd
    exit 0
fi

exit 100