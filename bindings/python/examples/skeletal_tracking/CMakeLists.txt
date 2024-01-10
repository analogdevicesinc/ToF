cmake_minimum_required(VERSION 3.10)

project(aditof-python-skeletal-tracking)

file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/skeletal_tracking.py" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/libs" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/models" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
