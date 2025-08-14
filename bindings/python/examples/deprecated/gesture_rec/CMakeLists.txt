cmake_minimum_required(VERSION 3.10)

project(aditof-python-gesture_rec)

file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/process.py" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/notebook.py" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")