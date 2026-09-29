cmake_minimum_required(VERSION 3.18)
project(UdCapCommunityDriverCore)

option(BUILD_TEST_TOOLS "Build Test Tool" OFF)
option(TEST_TOOLS_PRINT_RAW "Print raw data on test tool" OFF)
option(COPY_HIDAPI_DLL "Copy hidapi.dll to output directory" OFF)
option(USE_MANUAL_SERIAL_DRIVER "Use caller-provided serial callbacks instead of SerialReborn" OFF)

set(CMAKE_CXX_STANDARD 17)
if(CPM_CMAKE_PATH)
    include(${CPM_CMAKE_PATH})
else()
    file(
            DOWNLOAD
            https://github.com/cpm-cmake/CPM.cmake/releases/download/v0.40.8/CPM.cmake
            ${CMAKE_CURRENT_BINARY_DIR}/cmake/CPM.cmake
            EXPECTED_HASH SHA256=78ba32abdf798bc616bab7c73aac32a17bbd7b06ad9e26a6add69de8f3ae4791
    )
    include(${CMAKE_CURRENT_BINARY_DIR}/cmake/CPM.cmake)
endif()

if(NOT USE_MANUAL_SERIAL_DRIVER)
    add_subdirectory(serial-reborn)
endif()

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
        ${LIBUSBK_SRC}
)
if(USE_MANUAL_SERIAL_DRIVER)
    list(FILTER SRC_FILES EXCLUDE REGEX "[/\\\\]UdCapProbe\\.(cpp|h)$")
endif()
include_directories(src)
add_library(UdCapCommunityDriverCore STATIC ${SRC_FILES})

target_link_libraries(UdCapCommunityDriverCore PUBLIC AR1Linear Eigen nlohmann_json::nlohmann_json)
target_include_directories(UdCapCommunityDriverCore PUBLIC src)
if(USE_MANUAL_SERIAL_DRIVER)
    target_sources(UdCapCommunityDriverCore PRIVATE manual-serial/PortAccessor.cpp)
    target_include_directories(UdCapCommunityDriverCore PUBLIC manual-serial)
    target_compile_definitions(UdCapCommunityDriverCore PUBLIC USE_MANUAL_SERIAL_DRIVER=1)
else()
    target_link_libraries(UdCapCommunityDriverCore PUBLIC SerialReborn)
endif()
IF (WIN32 AND COPY_HIDAPI_DLL AND NOT USE_MANUAL_SERIAL_DRIVER)
    add_custom_command(TARGET UdCapCommunityDriverCore POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_BINARY_DIR}/serial-reborn/hidapi.dll"
            $<TARGET_FILE_DIR:UdCapCommunityDriverCore>)
ENDIF ()

IF (BUILD_TEST_TOOLS)
    enable_testing()
    project(UdCapCommunityDriverCoreTest)
    IF (TEST_TOOLS_PRINT_RAW)
        add_compile_definitions(TEST_TOOLS_PRINT_RAW)
    ENDIF ()
    if(USE_MANUAL_SERIAL_DRIVER)
        add_executable(UdCapManualSerialTest test/manual_serial.cpp)
        target_link_libraries(UdCapManualSerialTest PRIVATE UdCapCommunityDriverCore)
        add_test(NAME UdCapManualSerialTest COMMAND UdCapManualSerialTest)
    else()
        add_executable(UdCapCommunityDriverCoreTest test/main.cpp)
        target_link_libraries(UdCapCommunityDriverCoreTest PRIVATE UdCapCommunityDriverCore)
    endif()
    if(NOT USE_MANUAL_SERIAL_DRIVER)
        add_executable(UdCapCommunityDriverCorePacketTest test/packet.cpp)
        target_link_libraries(UdCapCommunityDriverCorePacketTest PRIVATE UdCapCommunityDriverCore)
        add_test(NAME UdCapCommunityDriverCorePacketTest COMMAND UdCapCommunityDriverCorePacketTest)
    endif()
    add_executable(UdCapCommunityDriverCoreCalibrationTest test/community_calibration.cpp src/CommunityV1Calibration.cpp)
    target_include_directories(UdCapCommunityDriverCoreCalibrationTest PRIVATE src)
    add_test(NAME UdCapCommunityDriverCoreCalibrationTest COMMAND UdCapCommunityDriverCoreCalibrationTest)
    add_executable(UdCapOfficialCalculationTest test/official_calculation.cpp src/OfficialCalibration.cpp)
    target_include_directories(UdCapOfficialCalculationTest PRIVATE src)
    target_link_libraries(UdCapOfficialCalculationTest PRIVATE AR1Linear)
    add_test(NAME UdCapOfficialCalculationTest COMMAND UdCapOfficialCalculationTest)
    add_executable(UdCapHandResponseTest test/hand_response.cpp)
    target_include_directories(UdCapHandResponseTest PRIVATE src)
    add_test(NAME UdCapHandResponseTest COMMAND UdCapHandResponseTest)
    if(NOT USE_MANUAL_SERIAL_DRIVER)
        add_executable(UdCapDeadlineSocketTest test/deadline_socket.cpp)
        target_include_directories(UdCapDeadlineSocketTest PRIVATE serial-reborn/src)
        target_link_libraries(UdCapDeadlineSocketTest PRIVATE Boost::asio)
        add_test(NAME UdCapDeadlineSocketTest COMMAND UdCapDeadlineSocketTest)
    endif()
IF (WIN32 AND COPY_HIDAPI_DLL AND NOT USE_MANUAL_SERIAL_DRIVER)
    add_custom_command(TARGET UdCapCommunityDriverCoreTest POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_BINARY_DIR}/hidapi.dll"
            $<TARGET_FILE_DIR:UdCapCommunityDriverCoreTest>)
ENDIF ()
ENDIF ()
