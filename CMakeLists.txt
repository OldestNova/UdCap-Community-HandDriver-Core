cmake_minimum_required(VERSION 3.18)
project(UdCapCommunityDriverCore)

option(BUILD_TEST_TOOLS "Build Test Tool" OFF)
option(TEST_TOOLS_PRINT_RAW "Print raw data on test tool" OFF)
option(COPY_HIDAPI_DLL "Copy hidapi.dll to output directory" OFF)

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

CPMAddPackage(
        NAME Eigen
        VERSION 3.4.0
        URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
        DOWNLOAD_ONLY YES
)

if(Eigen_ADDED)
    add_library(Eigen INTERFACE IMPORTED GLOBAL)
    target_include_directories(Eigen INTERFACE ${Eigen_SOURCE_DIR})
endif()
CPMAddPackage(
        NAME nlohmann_json
        GIT_TAG v3.12.0
        GITHUB_REPOSITORY nlohmann/json
)

file(GLOB_RECURSE SRC_FILES
        src/*.cpp
        src/*.h
        src/*.hpp
)
include_directories(src)
add_library(UdCapCommunityDriverCore STATIC ${SRC_FILES})
target_link_libraries(UdCapCommunityDriverCore PUBLIC SerialReborn AR1Linear Eigen nlohmann_json::nlohmann_json)
target_include_directories(UdCapCommunityDriverCore PUBLIC src)
IF (WIN32 AND COPY_HIDAPI_DLL)
    add_custom_command(TARGET UdCapCommunityDriverCore POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_BINARY_DIR}/serial-reborn/hidapi.dll"
            $<TARGET_FILE_DIR:UdCapCommunityDriverCore>)
ENDIF ()

IF (BUILD_TEST_TOOLS)
    project(UdCapCommunityDriverCoreTest)
    IF (TEST_TOOLS_PRINT_RAW)
        add_compile_definitions(TEST_TOOLS_PRINT_RAW)
    ENDIF ()
    add_executable(UdCapCommunityDriverCoreTest test/main.cpp)
    target_link_libraries(UdCapCommunityDriverCoreTest PRIVATE UdCapCommunityDriverCore)
IF (WIN32 AND COPY_HIDAPI_DLL)
    add_custom_command(TARGET UdCapCommunityDriverCoreTest POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_BINARY_DIR}/hidapi.dll"
            $<TARGET_FILE_DIR:UdCapCommunityDriverCoreTest>)
ENDIF ()
ENDIF ()