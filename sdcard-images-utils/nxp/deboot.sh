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

function setup_config() {

  # default configs
  TARGET_ARCH=arm64
  DISTRO_MIRROR='http://ports.ubuntu.com/ubuntu-ports'
  DISTRO_CODE=bionic
  USERNAME=analog
  PASSWORD=analog
  TZ_AREA=Etc
  TZ_CITY=UTC
  LOCALE_LANG=en_US.UTF-8
  ADD_LIST='git build-essential gcc autoconf nano vim'
  ADD_LIST_ST_3='i2c-tools v4l-utils rfkill wpasupplicant libtool libconfig-dev avahi-daemon'

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
usermod -a -G sudo,video,disk ${USERNAME}
echo -e "${PASSWORD}\n${PASSWORD}\n" | passwd ${USERNAME}

# overwrite apt source list
rm -f /etc/apt/sources.list
echo deb     ${DISTRO_MIRROR} ${DISTRO_CODE}          main restricted universe multiverse >> /etc/apt/sources.list
echo deb-src ${DISTRO_MIRROR} ${DISTRO_CODE}          main restricted universe multiverse >> /etc/apt/sources.list
echo deb     ${DISTRO_MIRROR} ${DISTRO_CODE}-updates  main restricted universe multiverse >> /etc/apt/sources.list
echo deb-src ${DISTRO_MIRROR} ${DISTRO_CODE}-updates  main restricted universe multiverse >> /etc/apt/sources.list
echo deb     ${DISTRO_MIRROR} ${DISTRO_CODE}-security main restricted universe multiverse >> /etc/apt/sources.list
echo deb-src ${DISTRO_MIRROR} ${DISTRO_CODE}-security main restricted universe multiverse >> /etc/apt/sources.list

apt update -y
apt upgrade -y

apt install -y ${ADD_LIST_ST_3}

#systemd configs
systemctl enable systemd-networkd.service
systemctl enable avahi-daemon.service
systemctl enable uvc-gadget.service

#sdk install
pushd /home/${USERNAME}
mkdir Workspace
pushd Workspace
git clone https://github.com/analogdevicesinc/aditof-sdk-rework.git
pushd aditof-sdk-rework/scripts/nxp/
chmod +x setup.sh
# ./setup.sh -y -b ../../build -d ../../deps -i /opt -j4
chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/Workspace
popd
popd
popd

EOF

  # Apply step1 overlay
  sudo cp -R ${SCRIPT_DIR}/patches/ubuntu_overlay/step1/* ${ROOTFS_TMP}/

  sudo mv stage3.sh ${ROOTFS_TMP}/tmp
  sudo chmod +x ${ROOTFS_TMP}/tmp/stage3.sh
  sudo chroot ${ROOTFS_TMP} /tmp/stage3.sh
  sudo rm -f ${ROOTFS_TMP}/tmp/stage3.sh
  
  sudo tar -xvf ${SCRIPT_DIR}/build/linux-imx/modules.tar -C ${ROOTFS_TMP}/

  # I don't know who creates empty .cache which is owned by root, remove it.
  sudo rm -rf ${ROOTFS_TMP}/home/${USERNAME}/.cache

  # Apply step3 overlay (configs)
  sudo cp -R ${SCRIPT_DIR}/patches/ubuntu_overlay/step3/* ${ROOTFS_TMP}/
}

function main() {

  setup_config
  
  cd ${OUTPUT_DIR}
  
  # create 3GB ext4 image
  dd if=/dev/zero of=rootfs.ext4 bs=1M count=3000
  mkdir -p ${ROOTFS_TMP}
  mkfs.ext4 rootfs.ext4
  sudo mount -o loop -o barrier=0 rootfs.ext4 ${ROOTFS_TMP}

  install_qemu

  # debootstrap 1st
  sudo debootstrap --arch=${TARGET_ARCH} --include="${INCLUDE_LIST}" --foreign ${DISTRO_CODE} ${ROOTFS_TMP} ${DISTRO_MIRROR}

  # debootstrap 2nd
  sudo chroot ${ROOTFS_TMP} /debootstrap/debootstrap --second-stage

  run_3rd_stage_script

  uninstall_qemu
  
  # cleanup
  sync
  sudo umount rootfs_tmp
  rm -R rootfs_tmp
}

main
