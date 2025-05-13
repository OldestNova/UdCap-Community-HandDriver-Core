cmake_minimum_required(VERSION 3.18)
project(AR1Linear)

set(CMAKE_CXX_STANDARD 17)

file(GLOB_RECURSE SRC_FILES
        src/*.cpp
        src/*.h
)

add_library(AR1Linear STATIC ${SRC_FILES})
