cmake_minimum_required(VERSION 3.18)
project(UdCapCommunityDriver)

set(CMAKE_CXX_STANDARD 17)
file(
        DOWNLOAD
        https://github.com/cpm-cmake/CPM.cmake/releases/download/v0.40.8/CPM.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/cmake/CPM.cmake
        EXPECTED_HASH SHA256=78ba32abdf798bc616bab7c73aac32a17bbd7b06ad9e26a6add69de8f3ae4791
)
include(${CMAKE_CURRENT_BINARY_DIR}/cmake/CPM.cmake)

add_subdirectory(serial-reborn)
include_directories(serial-reborn/src)

add_subdirectory(ar1_linear)
include_directories(ar1_linear/src)

file(GLOB_RECURSE SRC_FILES
    src/*.cpp
    src/*.h
)

add_executable(UdCapCommunityDriver ${SRC_FILES})
target_link_libraries(UdCapCommunityDriver PRIVATE SerialReborn AR1Linear)
IF (WIN32)
    add_custom_command(TARGET UdCapCommunityDriver POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_BINARY_DIR}/serial-reborn/hidapi.dll"
            $<TARGET_FILE_DIR:UdCapCommunityDriver>)
ENDIF ()