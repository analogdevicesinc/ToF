#!/bin/bash

# Function to validate version format

validate_version(){
	[[ "$1" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]
}

# Loop until version is entered
while true; do
	read -p "Enter version (format x.y.z): " version
	if validate_version "$version"; then
		break
	else
	   echo " Invalid format. Please enter version in x.y.z format (e.g., 1.0.0)"
	fi
done
# get the module path

read -p "Give path of module file: " module_file_path

if [ -z "$module_file_path" ]; then
	echo -e "\033[0;31mNo file path provided. Exiting.\033[0m"
	exit 1
fi

if [ ! -f "$module_file_path" ]; then
	echo -e "\033[0;31mFile does not exists: $module_file_path\033[0m"
	exit 1
fi

if [[ "$module_file_path" != *.ko ]]; then
	echo -e "\033[0;31mFile is not a .ko file. Exiting.\033[0m"
	exit 1
fi

read -p "Give the libs directory: " libs_path

if [ -d "$libs_path" ] && [ "$(ls -A "$libs_path")" ]; then
	# check all files are .so files
	if find "$libs_path" -type -f ! -name "*.so" | grep -q .; then
		echo "Folder contains files other that .so"
	else
		echo "Folder is not empty and contains only .so files"
	fi
else
	echo "Folder is either empty or does not exists."
fi

# Create Workspace directory
dir_name="Workspace-$version"
mkdir -p "$dir_name"

echo -e " \033[0;32mDirectory '$dir_name' created successfully.\033[0m"

cd $dir_name

# clone the ToF repo
git clone https://github.com/analogdevicesinc/ToF.git

cd ToF/

git submodule update --init --recursive

# delete git metadata
rm -rf .git

cd ../
cd ../

mkdir -p $dir_name/Tools 
# Copy the Tools folder to Workspace directory
cp -r $dir_name/ToF/sdcard-images-utils/nxp/patches/ubuntu_overlay/step3/home/analog/Workspace/Tools/* $dir_name/Tools/

if [ $? -eq 0 ]; then

	echo -e " \033[0;32mTools Folder is copied.\033[0m"
else
	echo -e " \033[0;31mCopy Failed.\033[0m"
fi

mkdir -p $dir_name/services
# Copy services files
cp -r $dir_name/ToF/sdcard-images-utils/nxp/patches/ubuntu_overlay/step1/usr/lib/systemd/system/*.service $dir_name/services

if [ $? -eq 0 ]; then

	echo -e " \033[0;32mservices Folder is copied.\033[0m"
else
	echo -e " \033[0;31mCopy Failed.\033[0m"
fi

# remove the irrelevant services
cd $dir_name/services

rm -f gunicorn.service uvc-gadget.service

cd ../../

# copy the libs folder
mkdir -p $dir_name/libs

cp  $libs_path/*.so $dir_name/libs

mkdir -p $dir_name/module

cp $module_file_path $dir_name/module

# Create requirements directory

mkdir -p $dir_name/requirements
cp -r $dir_name/ToF/Web-UI/requirements/* $dir_name/requirements

# make a zip file 

tar -cvzf $dir_name.tar.gz $dir_name/

# make sha-256 file
sha256sum $dir_name.tar.gz | awk '{print $1}' > $dir_name.tar.gz.sha256












