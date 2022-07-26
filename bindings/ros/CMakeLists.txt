cmake_minimum_required(VERSION 2.8.3)

if(UNIX)

  # set variables
  set(CATKIN_WS "${CMAKE_BINARY_DIR}/catkin_ws")
  set(ROS_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

  # create and run ROS setup script
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ros_setup.bash
                 ${CMAKE_BINARY_DIR}/tmp/ros_setup.bash)
  file(
    COPY ${CMAKE_BINARY_DIR}/tmp/ros_setup.bash
    DESTINATION ${CMAKE_BINARY_DIR}
    FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE)

  add_custom_target(aditof_ros_package)
  string(CONCAT OLD_CMAKE_INSTALL_PREFIX "\"${CMAKE_INSTALL_PREFIX}\"")
  string(CONCAT OLD_CMAKE_PREFIX_PATH "\"${CMAKE_PREFIX_PATH}\"")

  if("${ROS_HOME_DIR}" STREQUAL "")
    message(FATAL_ERROR "No ROS_HOME_DIR provided!")
  else()
    file(COPY ${CMAKE_BINARY_DIR}/../sdk/src/cameras/itof-camera/config DESTINATION "${ROS_HOME_DIR}")
  endif()

  add_custom_command(
    TARGET aditof_ros_package
    COMMAND ${CMAKE_BINARY_DIR}/ros_setup.bash "${OLD_CMAKE_INSTALL_PREFIX}"
            "${OLD_CMAKE_PREFIX_PATH}" "${LIBTOFI_LIBDIR_PATH}" ${VERSION} DEPENDS aditof)
  
endif()

