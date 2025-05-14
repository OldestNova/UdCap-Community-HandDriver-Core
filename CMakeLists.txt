cmake_minimum_required(VERSION 3.18)
project(UdCapCommunityDriverCore)

option(BUILD_TEST_TOOLS "Build Test Tool" OFF)

set(CMAKE_CXX_STANDARD 17)
file(
        DOWNLOAD
        https://github.com/cpm-cmake/CPM.cmake/releases/download/v0.40.8/CPM.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/cmake/CPM.cmake
        EXPECTED_HASH SHA256=78ba32abdf798bc616bab7c73aac32a17bbd7b06ad9e26a6add69de8f3ae4791
)
include(${CMAKE_CURRENT_BINARY_DIR}/cmake/CPM.cmake)

add_subdirectory(serial-reborn)

add_subdirectory(ar1_linear)

file(GLOB_RECURSE SRC_FILES
        src/*.cpp
        src/*.h
        src/*.hpp
)
include_directories(src)
add_library(UdCapCommunityDriverCore STATIC ${SRC_FILES})
target_link_libraries(UdCapCommunityDriverCore PUBLIC SerialReborn AR1Linear)
target_include_directories(UdCapCommunityDriverCore PUBLIC src)
IF (WIN32)
    add_custom_command(TARGET UdCapCommunityDriverCore POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_BINARY_DIR}/serial-reborn/hidapi.dll"
            $<TARGET_FILE_DIR:UdCapCommunityDriverCore>)
ENDIF ()

IF (BUILD_TEST_TOOLS)
    project(UdCapCommunityDriverCoreTest)
    add_executable(UdCapCommunityDriverCoreTest test/main.cpp)
    target_link_libraries(UdCapCommunityDriverCoreTest PRIVATE UdCapCommunityDriverCore)
    add_custom_command(TARGET UdCapCommunityDriverCoreTest POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_BINARY_DIR}/hidapi.dll"
            $<TARGET_FILE_DIR:UdCapCommunityDriverCoreTest>)
ENDIF ()