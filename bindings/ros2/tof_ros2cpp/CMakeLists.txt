cmake_minimum_required(VERSION 3.4)
project(tof_ros2cpp)
# Set Cmake standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
message("Build ROS2 binding for ADI TOF Camera")
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
#Finding Glog, Libwebsocket, Protobuf, Aditof and Tofi_compute libraries
list(APPEND CMAKE_PREFIX_PATH ${ADITOF_CMAKE_INSTALL_PREFIX}
     ${ADITOF_CMAKE_PREFIX_PATH})
list(APPEND CMAKE_PREFIX_PATH "/usr/local;/opt/glog;/opt/protobuf;/opt/websockets" ${ADITOF_CMAKE_PREFIX_PATH})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)

find_package(aditof REQUIRED)
find_library(TOFI_COMPUTE
  NAMES tofi_compute
  PATHS /usr/lib)


include_directories(include/${PROJECT_NAME} ${aditof_INCLUDE_DIRS})

file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_executable(tof_camera_node ${CMAKE_CURRENT_SOURCE_DIR}/src/tof_camera_node.cpp ${SOURCES})

target_link_libraries(tof_camera_node aditof::aditof)
ament_target_dependencies(tof_camera_node image_transport rclcpp std_msgs)
target_sources(tof_camera_node INTERFACE ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME})
add_library(tofi_compute SHARED IMPORTED)

install(TARGETS
tof_camera_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()


# Copying config files and depth compute libraries from sdk/config, sdk/lib folders to build/examples/first-frame and build/first-frame/Debug or Release folders    
add_custom_command( TARGET tof_camera_node POST_BUILD
COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/config ${CMAKE_CURRENT_SOURCE_DIR}/../../config
    COMMENT "Copying cfgs and libs to build config "
)