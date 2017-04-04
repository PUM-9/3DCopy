cmake_minimum_required(VERSION 3.6)
project(3DCopy)

set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SOURCE_FILES src/main.cpp src/Cli.cpp src/test_client.cpp)
add_executable(3DCopy ${SOURCE_FILES})

target_link_libraries(3DCopy ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
