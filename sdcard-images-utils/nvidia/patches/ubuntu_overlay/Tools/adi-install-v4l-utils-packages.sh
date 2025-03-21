#!/bin/bash

wget http://ftp.de.debian.org/debian/pool/main/libj/libjpeg-turbo/libjpeg62-turbo_2.1.5-2_arm64.deb

wget http://ftp.de.debian.org/debian/pool/main/v/v4l-utils/libv4lconvert0_1.22.1-5+b2_arm64.deb

wget http://ftp.de.debian.org/debian/pool/main/v/v4l-utils/libv4l2rds0_1.22.1-5+b2_arm64.deb

wget http://ftp.de.debian.org/debian/pool/main/v/v4l-utils/libv4l-0_1.22.1-5+b2_arm64.deb

wget http://ftp.de.debian.org/debian/pool/main/v/v4l-utils/v4l-utils_1.22.1-5+b2_arm64.deb

sudo dpkg -i libjpeg62-turbo_2.1.5-2_arm64.deb

sudo dpkg -i libv4lconvert0_1.22.1-5+b2_arm64.deb

sudo dpkg -i libv4l2rds0_1.22.1-5+b2_arm64.deb

sudo dpkg -i libv4l-0_1.22.1-5+b2_arm64.deb

sudo dpkg -i v4l-utils_1.22.1-5+b2_arm64.deb

sudo rm -rf *.deb