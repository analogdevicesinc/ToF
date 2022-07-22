add_executable(file_loopback_c file_loopback.c)
add_executable(file_writer_c file_writer.c)

target_link_libraries(file_loopback_c fsf_wrapper fsf_common fsf)
target_link_libraries(file_writer_c fsf_wrapper fsf_common fsf)
