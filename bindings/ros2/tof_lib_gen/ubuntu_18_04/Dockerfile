FROM ubuntu:18.04 AS stage1
 
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
    && apt-get install wget

# Install cmake version v3.20
RUN \
    wget https://github.com/Kitware/CMake/releases/download/v3.20.0/cmake-3.20.0.tar.gz \
    && tar -zvxf cmake-3.20.0.tar.gz \
    && cd cmake-3.20.0 \
    && ./bootstrap \
    && make -j8 \
    && make install .

RUN \
mkdir tof_lib && cd tof_lib \
&& mkdir DEBIAN && cd DEBIAN \
&& echo 'Package:tof-lib\n\
Version:1.0.1\n\
Maintainer:Analog Devices Inc\n\
Architecture:amd64\n\
Description:ROS_Wrapper_for_ADIs_ToF_SDK' > control

ENV DESTDIR_TOF_LIB=/tof_lib

#Install Glog
RUN \
    git clone -b v0.6.0 --depth 1 https://github.com/google/glog \
    && cd glog \
    && mkdir build_0_6_0 && cd build_0_6_0 \
    && cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX=/opt/glog .. \
    && cmake --build . --target install \
    && cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX=$DESTDIR_TOF_LIB/opt/glog .. \
    && cmake --build . --target install


#Install Libwebsoocket
RUN \
    apt-get install libssl-dev \
    && git clone --branch v3.1-stable --depth 1 https://github.com/warmcat/libwebsockets \
    && cd libwebsockets \
    && mkdir build_3_1 && cd build_3_1 \
    && cmake -DLWS_WITH_SSL=OFF -DLWS_STATIC_PIC=ON -DCMAKE_INSTALL_PREFIX=/opt/websockets .. \
    && cmake --build . --target install \
    && cmake -DLWS_WITH_SSL=OFF -DLWS_STATIC_PIC=ON -DCMAKE_INSTALL_PREFIX=$DESTDIR_TOF_LIB/opt/websockets .. \
    && cmake --build . --target install

#Install Protobuf
RUN \
    git clone --branch v3.9.0 --depth 1 https://github.com/protocolbuffers/protobuf \
    && cd protobuf \
    && mkdir build_3_9_0 && cd build_3_9_0 \
    && cmake -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_INSTALL_PREFIX=/opt/protobuf ../cmake \
    && cmake --build . --target install \
    && cmake -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_INSTALL_PREFIX=$DESTDIR_TOF_LIB/opt/protobuf ../cmake \
    && cmake --build . --target install

RUN \
    git clone https://github.com/analogdevicesinc/ToF \
    && cd ToF \
    && mkdir build && cd build \
    && cmake -DUSE_DEPTH_COMPUTE_OPENSOURCE=1 -DWITH_NETWORK=1 -DWITH_EXAMPLES=off -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" -DCMAKE_INSTALL_PREFIX=$DESTDIR_TOF_LIB/usr/ .. \
    && cmake --build . --target install 

#Generating dpkg package
RUN \
    dpkg-deb --build tof_lib

FROM scratch AS export-stage
COPY --from=stage1 tof_lib.deb .

