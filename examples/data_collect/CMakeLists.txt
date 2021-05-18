cmake_minimum_required(VERSION 3.10)
project(data_collect)

set(DOCOPT_LIBRARY third_party/doc_opt)
set(FSF_LIBRARY adi/FSF)

include_directories(
    ${DOCOPT_LIBRARY}
    ${FSF_LIBRARY}/include
    ${FSF_LIBRARY}/include/fsf
    ${FSF_LIBRARY}/src
)

set( SOURCES
${DOCOPT_LIBRARY}/docopt.cpp
${FSF_LIBRARY}/src/fsf.cpp
${FSF_LIBRARY}/src/fsf_common.cpp
${FSF_LIBRARY}/src/fsf_wrapper.cpp
)

add_executable(${PROJECT_NAME} main.cpp ${SOURCES})

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 11)

if( WIN32 )
        set(OS_SPECIFIC_DIR windows)
elseif( APPLE )
        set(OS_SPECIFIC_DIR macos)    
elseif( DRAGONBOARD )
        set(OS_SPECIFIC_DIR ${TARGET_STRING})
        set(TARGET_SPECIFIC_DIR dragonboard)
elseif ( RASPBERRYPI )
        set(OS_SPECIFIC_DIR ${TARGET_STRING})
        set(TARGET_SPECIFIC_DIR raspberrypi)
elseif ( UNIX )
        set(OS_SPECIFIC_DIR linux)
else()
    message(FATAL_ERROR "Platform not supported")
endif()

# Copying config files and depth compute libraries from sdk/config, sdk/lib folders to build/examples/first-frame and build/first-frame/Debug or Release folders    
add_custom_command( TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/../../sdk/src/cameras/itof-camera/config $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
    # COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/../../sdk/src/cameras/itof-camera/config $<TARGET_FILE_DIR:${PROJECT_NAME}>/../
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${LIBTOFI_LIBDIR_PATH} $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
    # COMMAND ${CMAKE_COMMAND} -E copy_directory LIBTOFI_LIBDIR_PATH $<TARGET_FILE_DIR:${PROJECT_NAME}>/../
    COMMENT "Copying cfgs and libs to build examples "
)
if( WIN32 )
add_custom_command( TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_BINARY_DIR}/sdk/$<CONFIGURATION>" $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
    # COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_BINARY_DIR}/sdk/$<CONFIGURATION>" $<TARGET_FILE_DIR:${PROJECT_NAME}>/../
)
else()
add_custom_command( TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_BINARY_DIR}/sdk/" $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
    # COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_BINARY_DIR}/sdk/" $<TARGET_FILE_DIR:${PROJECT_NAME}>/../
)
endif()

if ( WIN32 )
   if (MSVC)
      set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /STACK:4194304")
   else ()
      set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--stack,4194304")
   endif()
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE aditof)
