FROM ubuntu:20.04 AS stage1
 
LABEL maintainer="Robert Budai, <robert.budai@analog.com>"
ARG DEBIAN_FRONTEND=noninteractive

# Install updates to base image
RUN \
    apt-get update -y \
    && apt-get install -y git \
    && apt-get install -y cmake \
    && apt-get install -y build-essential \
    && apt-get install -y apt-utils \
    && apt-get install -y libssl-dev \
    && apt-get install wget \
    && apt-get -y install sudo

ENV DESTDIR_TOF_LIB=/tof_lib

SHELL ["/bin/bash", "-c"]

RUN \
mkdir tof_lib && cd tof_lib \
&& mkdir DEBIAN && cd DEBIAN \
&& echo $'Package:tof-lib\nVersion:1.0.1\nMaintainer:Analog Devices Inc\nArchitecture:amd64\nDescription:ROS_Wrapper_for_ADIs_ToF_SDK' > control

RUN \
    git clone https://github.com/analogdevicesinc/ToF \
    && cd ToF && git checkout tags/v4.2.0 \
    && source scripts/nxp/utils.sh && get_deps_source_code ./ 
RUN \
    cd ToF \
    && source scripts/nxp/utils.sh && build_and_install_glog ./glog /opt/glog 
RUN \
    cd ToF \
    && source scripts/nxp/utils.sh && build_and_install_protobuf ./protobuf /opt/protobuf 
RUN \
    cd ToF \
    && source scripts/nxp/utils.sh && build_and_install_websockets ./libwebsockets /opt/websockets 
RUN \
    cd ToF \
    && mkdir build && cd build \
    && cmake -DUSE_DEPTH_COMPUTE_OPENSOURCE=1 -DWITH_NETWORK=1 -DWITH_EXAMPLES=0 -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" -DCMAKE_INSTALL_PREFIX=$DESTDIR_TOF_LIB/usr/ .. \
    && cmake --build . --target install 

RUN \
    cd ToF \
    && source scripts/nxp/utils.sh && get_deps_source_code ./ \
    && source scripts/nxp/utils.sh && build_and_install_glog ./glog $DESTDIR_TOF_LIB/opt/glog \
    && source scripts/nxp/utils.sh && build_and_install_protobuf ./protobuf $DESTDIR_TOF_LIB/opt/protobuf \
    && source scripts/nxp/utils.sh && build_and_install_websockets ./libwebsockets $DESTDIR_TOF_LIB/opt/websockets 

#Generating dpkg package
RUN \
    dpkg-deb --build tof_lib

FROM scratch AS export-stage
COPY --from=stage1 tof_lib.deb .
