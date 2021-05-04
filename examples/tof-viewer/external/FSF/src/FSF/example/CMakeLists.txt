add_executable(file_loopback file_loopback.cpp)
add_executable(file_writer file_writer.cpp)

target_link_libraries(file_loopback fsf_common fsf)
target_link_libraries(file_writer fsf_common fsf)
