add_library(fsf fsf.cpp)
add_library(fsf_common fsf_common.cpp)
add_library(fsf_wrapper fsf_wrapper.cpp)

target_include_directories(fsf PUBLIC ../include .)
target_include_directories(fsf_common PUBLIC ../include .)
target_include_directories(fsf_wrapper PUBLIC ../include .)
