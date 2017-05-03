cmake_minimum_required(VERSION 3.6)
project(3DCopy)

set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.3 REQUIRED)
find_package(Boost 1.64 COMPONENTS program_options filesystem)

if (NOT Boost_FOUND)
    message(WARNING "Boost (version >= 1.64) required for slicer support.")
    find_package(Boost 1.40 COMPONENTS program_options filesystem REQUIRED)
else()
    add_definitions(-DENABLE_SLICER)
endif (NOT Boost_FOUND)

message(STATUS "Setting up BOOST")
message(STATUS " Includes - ${Boost_INCLUDE_DIRS}")
message(STATUS " Library  - ${Boost_LIBRARY_DIRS}")

include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SOURCE_FILES src/main.cpp src/Cli.cpp src/Mesh.cpp src/Registration.cpp src/Slicer.cpp)
add_executable(3DCopy ${SOURCE_FILES})

target_link_libraries(3DCopy ${PCL_LIBRARIES} ${Boost_LIBRARIES})
