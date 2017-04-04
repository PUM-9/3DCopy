cmake_minimum_required(VERSION 3.6)
project(3DCopy)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES src/main.cpp src/Cli.cpp)
add_executable(3DCopy ${SOURCE_FILES})