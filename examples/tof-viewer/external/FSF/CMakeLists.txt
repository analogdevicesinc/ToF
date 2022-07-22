# Adding fsf
if (NOT TARGET fsf)
    set(fsf_SOURCE_DIR src)
    add_library(fsf STATIC
                ${fsf_SOURCE_DIR}/FSF/src/fsf.cpp                
    )

    #target_include_directories(fsf PUBLIC
    #    ${fsf_SOURCE_DIR}
    #    ${fsf_SOURCE_DIR}/examples
    #    ${fsf_SOURCE_DIR}/examples/libs/gl3w
    #    ${fsf_SOURCE_DIR}/examples/libs/glfw/include
    #)
else()
    message(STATUS "fsf is already a target. Skipping adding it twice")
endif()

target_include_directories(fsf PUBLIC .. src/FSF/include)
add_library(fsf::fsf ALIAS fsf)

# Adding fsf_common
if (NOT TARGET fsf_common)
    set(fsf_common_SOURCE_DIR src)
    add_library(fsf_common STATIC
                ${fsf_common_SOURCE_DIR}/FSF/src/fsf_common.cpp			
    )
else()
    message(STATUS "fsf_common is already a target. Skipping adding it twice")
endif()

target_include_directories(fsf_common PUBLIC .. 
							src/FSF/src
							src/FSF/include
							)
add_library(fsf_common::fsf_common ALIAS fsf_common)