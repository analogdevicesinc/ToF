cmake_minimum_required(VERSION 3.0)

project(aditof-python-dnn)

# Download MobileNetSSD_deploy prototxt and caffemodel
set(PROTOTEXT_FILE "MobileNetSSD_deploy.prototxt")
set(PROTOTEXT_URL "https://raw.githubusercontent.com/djmv/MobilNet_SSD_opencv/master/MobileNetSSD_deploy.prototxt")
set(PROTOTEXT_PATH "${CMAKE_CURRENT_BINARY_DIR}/${PROTOTEXT_FILE}")

set(MODEL_FILE "MobileNetSSD_deploy.caffemodel")
set(MODEL_URL "https://github.com/djmv/MobilNet_SSD_opencv/blob/master/MobileNetSSD_deploy.caffemodel?raw=true")
set(MODEL_PATH "${CMAKE_CURRENT_BINARY_DIR}/${MODEL_FILE}")


if(NOT EXISTS "${PROTOTEXT_PATH}")
	message("Downloading ${PROTOTEXT_FILE} into ${CMAKE_CURRENT_BINARY_DIR}")
    file(DOWNLOAD "${PROTOTEXT_URL}" "${PROTOTEXT_PATH}")
endif()

if(NOT EXISTS "${MODEL_PATH}")
	message("Downloading ${MODEL_FILE} into ${CMAKE_CURRENT_BINARY_DIR}")
    file(DOWNLOAD "${MODEL_URL}" "${MODEL_PATH}")
endif()

file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/dnn.py" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")