#!/bin/bash
#
# deboot.sh
# script to build Ubuntu rootfs (for arm64, armhf, powerpc, ppc64el)
#
# Copyright 2017 knotdevel
# Released under the MIT license
# http://opensource.org/licenses/mit-license.php
#
#
# Prerequisite:
#
# 1. Setup Ubuntu 16.04 64bit desktop machine connected to the Internet
# 2. Upgrade the software
#      $ sudo apt update
#      $ sudo apt upgrade
# 3. Install below packages
#      $ sudo apt install debootstrap qemu-user-static
# 4. Download this script
#      $ git clone https://gist.github.com/knotdevel/bc5b8d547edd374bf0017fe16bf751bc deboot
#      $ cd deboot
#
#
# Usage:
#
# $ ./deboot.sh
#
#
# Configuration:
# If you want to change the configuration, create a file 'config' with below contents
# in the same directory.
#
# # example of the config
# TARGET_ARCH=arm64
# DISTRO_MIRROR='http://jp.ports.ubuntu.com/ubuntu-ports'
# DISTRO_CODE=xenial
# USERNAME=myusername
# PASSWORD=complex!password#123
# TZ_AREA=Asia
# TZ_CITY=Tokyo
# LOCALE_LANG=ja_JP.UTF-8
# ADD_LIST='git build-essential ubuntu-desktop'
#
#
# Output:
#
# apt cache file    : apt-cache-xenial-arm64_<date>_<time>.tgz
# rootfs cache file : rfs-cache-xenial-arm64_<date>_<time>.tgz
# final rootfs      : rootfs-xenial-arm64_<date>_<time>.tgz
#
#

set -e
export LC_ALL=C
export LANGUAGE=C
export LANG=C

BRANCH=$1
LIBADITOF_BRANCH=$2

function setup_config() {

  # default configs
  TARGET_ARCH=arm64
  DISTRO_MIRROR=${DISTRO_MIRROR:='http://ports.ubuntu.com/ubuntu-ports'}
  DISTRO_CODE=focal
  USERNAME=analog
  PASSWORD=analog
  TZ_AREA=Etc
  TZ_CITY=UTC
  LOCALE_LANG=en_US.UTF-8
  ADD_LIST='git build-essential gcc autoconf nano vim parted flex bison'
  ADD_LIST_ST_3='i2c-tools v4l-utils rfkill wpasupplicant libtool libconfig-dev avahi-daemon htpdate openssh-server iperf3 bc python3-dev python3-pip python3-matplotlib gunicorn python3-gevent python3-flask python3-tz unzip'
  ROS_DEP='software-properties-common'

  # output example of the config file
#  cat <<EOF>config-example
# config file example
# modify below, then rename to 'config'
#TARGET_ARCH=arm64
#DISTRO_MIRROR='http://jp.ports.ubuntu.com/ubuntu-ports'
#DISTRO_CODE=distro
#USERNAME=myusername
#PASSWORD=complex!password#123
#TZ_AREA=Asia
#TZ_CITY=Tokyo
#LOCALE_LANG=ja_JP.UTF-8
#ADD_LIST='git build-essential ubuntu-desktop'
#EOF

  # user overrides the default configs
  if [ -f config ]; then
    source config
  fi

  # check configs
  echo TARGET_ARCH : ${TARGET_ARCH}
  echo MIRROR      : ${DISTRO_MIRROR}
  echo CODE        : ${DISTRO_CODE}
  echo USERNAME    : ${USERNAME}
  echo PASSWORD    : ${PASSWORD}
  echo TZ_AREA     : ${TZ_AREA}
  echo TZ_CITY     : ${TZ_CITY}
  echo LANG        : ${LOCALE_LANG}
  echo ADD_LIST    : ${ADD_LIST}
  echo ADD_LIST_STAGE_3 : ${ADD_LIST_ST_3}
  echo ROS_DEP : ${ROS_DEP}

  # language pack
  LANG_PACK=language-pack-${LOCALE_LANG%%_*}-base
  INCLUDE_LIST=${ADD_LIST}" "${LANG_PACK}
  echo INCLUDE_LIST: ${INCLUDE_LIST}

  #INCLUDE_LIST=language-pack-en-base
  #INCLUDE_LIST=language-pack-ja-base
  #INCLUDE_LIST=language-pack-${LOCALE_LANG:0:2}-base
  #EXCLUDE_LIST=lightdm-gtk-greeter
  #COMPONENT_LIST=main,restricted,universe,multiverse

  SCRIPT_DIR=$(cd $(dirname $0);pwd)
  OUTPUT_DIR=${SCRIPT_DIR}/build/ubuntu
  ROOTFS_TMP=${SCRIPT_DIR}/build/ubuntu/rootfs_tmp
  ROOTFS_CACHE=rfs-cache-${DISTRO_CODE}-${TARGET_ARCH}
  ROOTFS=rootfs-${DISTRO_CODE}-${TARGET_ARCH}
  APT_CACHE=apt-cache-${DISTRO_CODE}-${TARGET_ARCH}
  TIME_STAMP=$(echo `date +%Y%m%d_%H%M%S`)

  case ${TARGET_ARCH} in
  "arm64")
    QEMU_ARCH=aarch64
    ;;
  "armhf")
    QEMU_ARCH=arm
    ;;
  "ppc64el")
    QEMU_ARCH=ppc64le
    ;;
  "powerpc")
    QEMU_ARCH=ppc
    ;;
  *)
    exit 1
    ;;
  esac
}

function extract_apt_cache() {
  if [ -f ${APT_CACHE}.tgz ]; then
    sudo mkdir -p ${ROOTFS_TMP}/var/cache
    sudo tar zxf ${APT_CACHE}.tgz -C ${ROOTFS_TMP}/var/cache/
  fi
}

function archive_apt_cache() {
  sudo rm -f ${APT_CACHE}.tgz
  cd ${ROOTFS_TMP}/var/cache
  # archive apt cache
  sudo tar zcf ${OUTPUT_DIR}/${APT_CACHE}_${TIME_STAMP}.tgz apt/
  # remove apt cache from rootfs to reduce rootfs image size
  sudo rm -rf apt/
  cd -
  ln -s ${APT_CACHE}_${TIME_STAMP}.tgz ${APT_CACHE}.tgz
}

function install_qemu() {
  sudo mkdir -p ${ROOTFS_TMP}/usr/bin
  sudo cp /usr/bin/qemu-${QEMU_ARCH}-static ${ROOTFS_TMP}/usr/bin
}

function uninstall_qemu() {
  sudo rm ${ROOTFS_TMP}/usr/bin/qemu-${QEMU_ARCH}-static
  echo ;
}

function archive_rootfs_base() {
  rm -f ${ROOTFS_CACHE}.tgz
  cd ${ROOTFS_TMP}
  sudo tar zcf ${OUTPUT_DIR}/${ROOTFS_CACHE}_${TIME_STAMP}.tgz .
  cd -
  ln -s ${ROOTFS_CACHE}_${TIME_STAMP}.tgz ${ROOTFS_CACHE}.tgz
}

function archive_rootfs() {
  rm -f ${ROOTFS}.tgz
  cd ${ROOTFS_TMP}
  sudo tar zcf ${OUTPUT_DIR}/${ROOTFS}_${TIME_STAMP}.tgz .
  cd -
  ln -s ${ROOTFS}_${TIME_STAMP}.tgz ${ROOTFS}.tgz
}

function run_3rd_stage_script() {
  # gen 3rd stage script
  cat<<EOF>stage3.sh
#!/bin/bash -ex

# set locale
echo 'LANG="${LOCALE_LANG}"' > /etc/default/locale
locale-gen ${LOCALE_LANG}
dpkg-reconfigure -f noninteractive locales

# set timezone
echo "tzdata tzdata/Areas select ${TZ_AREA}" > /tmp/tmptz
echo "tzdata tzdata/Zones/${TZ_AREA} select ${TZ_CITY}" >> /tmp/tmptz
debconf-set-selections /tmp/tmptz
rm -f /etc/timezone
rm -f /etc/localtime
dpkg-reconfigure -f noninteractive tzdata
rm -f /tmp/tmptz

# set default hostname
echo aditof > /etc/hostname

# create user and passwd
useradd -m -d /home/${USERNAME} -s /bin/bash ${USERNAME}
groupadd i2c
groupadd gpio
usermod -a -G sudo,video,disk,i2c,gpio ${USERNAME}
echo -e "${PASSWORD}\n${PASSWORD}\n" | passwd ${USERNAME}


# overwrite apt source list
rm -f /etc/apt/sources.list
echo deb     ${DISTRO_MIRROR} ${DISTRO_CODE}          main universe >> /etc/apt/sources.list
echo deb-src ${DISTRO_MIRROR} ${DISTRO_CODE}          main universe >> /etc/apt/sources.list
echo deb     ${DISTRO_MIRROR} ${DISTRO_CODE}-updates  main universe >> /etc/apt/sources.list
echo deb-src ${DISTRO_MIRROR} ${DISTRO_CODE}-updates  main universe >> /etc/apt/sources.list
echo deb     ${DISTRO_MIRROR} ${DISTRO_CODE}-security main universe >> /etc/apt/sources.list
echo deb-src ${DISTRO_MIRROR} ${DISTRO_CODE}-security main universe >> /etc/apt/sources.list

apt update -y
apt upgrade -y
apt-get update -y
apt-get upgrade -y

apt-get install -y ${ROS_DEP}

apt install -y ${ADD_LIST_ST_3}

#systemd configs
systemctl enable systemd-networkd.service
systemctl enable avahi-daemon.service
systemctl enable usb-gadget.service
systemctl enable network-gadget.path
systemctl enable uvc-gadget.path
systemctl enable adi-backup.service
systemctl enable gunicorn.service

#set default python3
update-alternatives --install /usr/bin/python python /usr/bin/python3 1

#sdk install
pushd /home/${USERNAME}
mkdir Workspace
pushd Workspace

if ls /*.so 1> /dev/null 2>&1; then
	mkdir libs
	sudo cp /*.so /home/${USERNAME}/Workspace/libs 1> /dev/null 2>&1
	sudo rm -rf /*.so 1> /dev/null 2>&1
fi
TOF_GIT_LOCATION=${TOF_GIT_LOCATION:=https://github.com/analogdevicesinc/ToF.git}
echo "Clone ToF source code from: $TOF_GIT_LOCATION"
git clone ${TOF_GIT_LOCATION}
if [ -n ${BRANCH} ]; then
	echo "Checkout to Branch: ${BRANCH}"
	pushd ToF
	git checkout ${BRANCH}
	popd
fi
pushd ToF
git submodule update --init --recursive
popd
if [ -n ${LIBADITOF_BRANCH} ]; then
	echo "Checkout to Branch: ${LIBADITOF_BRANCH}"
	pushd ToF
  pushd libaditof
	git checkout ${LIBADITOF_BRANCH}
	popd
  popd
fi

pushd ToF/scripts/nxp/
chmod +x setup.sh
./setup.sh -y -b ../../build -j4
popd
#cp ToF/build/apps/uvc-app/uvc-app /usr/share/systemd/
cp ToF/build/apps/server/aditof-server /usr/share/systemd/
popd
popd
chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/Workspace

#copy executables to bin folder
mkdir /home/${USERNAME}/Workspace/bin
cp /home/${USERNAME}/Workspace/ToF/build/examples/data_collect/data_collect /home/${USERNAME}/Workspace/bin
cp /home/${USERNAME}/Workspace/ToF/build/examples/first-frame/first-frame /home/${USERNAME}/Workspace/bin
mv /home/${USERNAME}/Workspace/ToF/sdcard-images-utils/nxp/patches/ubuntu_overlay/step1/usr/share/systemd/*.sh /home/${USERNAME}/Workspace/bin

pushd /home/${USERNAME}/Workspace/bin
chmod +x ros_install_noetic.sh
echo "3" | ./ros_install_noetic.sh
popd

#generate licences file
tail -n 10000 /usr/share/doc/*/copyright > /licenses.txt
EOF

  # Apply step1 overlay
  sudo cp -R ${SCRIPT_DIR}/patches/ubuntu_overlay/step1/* ${ROOTFS_TMP}/

  sudo mv stage3.sh ${ROOTFS_TMP}/tmp
  sudo chmod +x ${ROOTFS_TMP}/tmp/stage3.sh
  sudo chroot ${ROOTFS_TMP} /tmp/stage3.sh
  sudo rm -f ${ROOTFS_TMP}/tmp/stage3.sh
  
  sudo tar -xvf ${SCRIPT_DIR}/build/linux-imx/modules.tar -C ${ROOTFS_TMP}/usr/

  # Move the linux-imx kernel source to the NXP  
  sudo mkdir -p ${SCRIPT_DIR}/build/linux-imx_tmp
  sudo cp -Rf ${SCRIPT_DIR}/build/linux-imx/* ${SCRIPT_DIR}/build/linux-imx_tmp/
  sudo cp ${SCRIPT_DIR}/build/linux-imx/.config ${SCRIPT_DIR}/build/linux-imx_tmp/
  pushd ${SCRIPT_DIR}/build/linux-imx_tmp/
  sudo make clean
  popd

  sudo mkdir -p ${ROOTFS_TMP}/usr/src/linux-imx/
  sudo cp -Rf ${SCRIPT_DIR}/build/linux-imx_tmp/* ${ROOTFS_TMP}/usr/src/linux-imx/
  sudo cp ${SCRIPT_DIR}/build/linux-imx_tmp/.config ${ROOTFS_TMP}/usr/src/linux-imx/

  # I don't know who creates empty .cache which is owned by root, remove it.
  sudo rm -rf ${ROOTFS_TMP}/home/${USERNAME}/.cache

  # Apply step3 overlay (configs)
  sudo cp -R ${SCRIPT_DIR}/patches/ubuntu_overlay/step3/* ${ROOTFS_TMP}/

}

function create_ui_setup(){
	cat<<EOF>web_ui_setup.sh
#!/bin/bash

pushd /home/${USERNAME}

sudo mv /tmp/ros_temp/* /home/${USERNAME}/Workspace
chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/Workspace/catkin_ws

sudo mv Workspace/ToF/Web-UI/requirements/ /home/${USERNAME}/
sudo mv Workspace/ToF/Web-UI/web-1.0.0/ /home/${USERNAME}/

sudo  ln -s  /home/${USERNAME}/web-1.0.0 web
chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/web

sudo mkdir ADSD3500-firmware-5.2.5
sudo mv /tmp/fw_temp/* ADSD3500-firmware-5.2.5/
chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/ADSD3500-firmware-5.2.5
sudo ln -s /home/${USERNAME}/ADSD3500-firmware-5.2.5 ADSD3500-firmware
chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/ADSD3500-firmware
sudo mv Workspace/  Workspace-6.0.0
sudo ln -s /home/${USERNAME}/Workspace-6.0.0 Workspace
chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/Workspace


#copy service files to services folder
mkdir /home/${USERNAME}/Workspace/services
mv /usr/lib/systemd/system/adi-backup.service /home/${USERNAME}/Workspace/services
mv /usr/lib/systemd/system/adi-tof.service /home/${USERNAME}/Workspace/services
mv /usr/lib/systemd/system/network-gadget.service /home/${USERNAME}/Workspace/services
mv /usr/lib/systemd/system/usb-gadget.service /home/${USERNAME}/Workspace/services
chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/Workspace/services

# link the services in Workspace
sudo ln -s /home/${USERNAME}/Workspace/services/adi-backup.service /usr/lib/systemd/system/adi-backup.service
sudo ln -s /home/${USERNAME}/Workspace/services/adi-tof.service /usr/lib/systemd/system/adi-tof.service
sudo ln -s /home/${USERNAME}/Workspace/services/network-gadget.service /usr/lib/systemd/system/network-gadget.service
sudo ln -s /home/${USERNAME}/Workspace/services/usb-gadget.service /usr/lib/systemd/system/usb-gadget.service

#copy the driver build in modules folderAdd commentMore actions
mkdir /home/${USERNAME}/Workspace/module
mv /usr/lib/modules/5.10.72-*/kernel/drivers/media/i2c/adsd3500.ko /home/${USERNAME}/Workspace/module
sudo ln -s /home/${USERNAME}/Workspace/module/adsd3500.ko /usr/lib/modules/5.10.72-*/kernel/drivers/media/i2c/
chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/Workspace/module
rm -rf /tmp/ros_temp
rm -rf /tmp/fw_temp

EOF
     mkdir -p ${ROOTFS_TMP}/tmp/ros_temp
     mkdir -p ${ROOTFS_TMP}/tmp/fw_temp
     sudo cp -r ${SCRIPT_DIR}/ROS/* ${ROOTFS_TMP}/tmp/ros_temp/ &&  echo "ROS1 copied successfully" >> ${SCRIPT_DIR}/ROS/log.txt
     sudo cp -r ${SCRIPT_DIR}/Firmware/* ${ROOTFS_TMP}/tmp/fw_temp/ &&  echo "Firmware copied successfully" >> ${SCRIPT_DIR}/Firmware/log.txt
     sudo mv web_ui_setup.sh ${ROOTFS_TMP}/tmp
     sudo chmod +x ${ROOTFS_TMP}/tmp/web_ui_setup.sh
     sudo chroot ${ROOTFS_TMP} /tmp/web_ui_setup.sh
     sudo rm -f ${ROOTFS_TMP}/tmp/web_ui_setup.sh
}

function main() {

  echo ${BRANCH}
  setup_config
  
  cd ${OUTPUT_DIR}
  
  # create 8GB ext4 image
  dd if=/dev/zero of=rootfs.ext4 bs=1M count=8000
  mkdir -p ${ROOTFS_TMP}
  mkfs.ext4 rootfs.ext4
  sudo mount -o loop -o barrier=0 rootfs.ext4 ${ROOTFS_TMP}

  install_qemu

  # debootstrap 1st
  sudo debootstrap --arch=${TARGET_ARCH} --include="${INCLUDE_LIST}" --foreign ${DISTRO_CODE} ${ROOTFS_TMP} ${DISTRO_MIRROR}
  # copy libs to ${SCRIPT_DIR}/build/ubuntu/rootfs_tmp/home/temp(Make sure to add a path to .so files here)

  
  # debootstrap 2nd
  sudo chroot ${ROOTFS_TMP} /debootstrap/debootstrap --second-stage

  run_3rd_stage_script

  # for web-ui
  create_ui_setup
  
  uninstall_qemu
  #Adding rsz script to .bashrc
  sudo cat ${ROOTFS_TMP}/home/bashrc_extension >> ${ROOTFS_TMP}/home/${USERNAME}/.bashrc
  sudo cat ${ROOTFS_TMP}/home/${USERNAME}/.bashrc >> ${SCRIPT_DIR}/build/log_rc.txt
  sudo rm -rf ${ROOTFS_TMP}/home/bashrc_extension 1> /dev/null 2>&1

  
  # cleanup
  sync
  sudo umount rootfs_tmp
  rm -R rootfs_tmp
}

main
