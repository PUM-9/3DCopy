cmake_minimum_required(VERSION 3.7.1)
project(3DCopy)

set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.3 REQUIRED)
find_package(Boost 1.40 COMPONENTS program_options REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SOURCE_FILES src/main.cpp src/Cli.cpp src/Mesh.cpp src/Registration.cpp)
add_executable(3DCopy ${SOURCE_FILES})

target_link_libraries(3DCopy ${PCL_LIBRARIES} ${Boost_LIBRARIES})
