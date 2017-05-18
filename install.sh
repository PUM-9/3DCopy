#!/bin/sh

echo "Installing dependecies"
{
    # Install basic build dependencies
    apt-get install make g++ libboost-all-dev -y

    # Install PCL
    add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl -y
    apt-get update
    apt-get install libpcl-all -y

} > /dev/null 2>&1

echo "Creating build dir"
mkdir -p build
cd build

echo "Downloading cmake version 3.8.1"
{
    echo "hej"
    # Download recent version of cmake
    wget https://cmake.org/files/v3.8/cmake-3.8.1-Linux-x86_64.sh
    chmod +x cmake-3.8.1-Linux-x86_64.sh
    ./cmake-3.8.1-Linux-x86_64.sh --include-subdir
    rm cmake-3.8.1-Linux-x86_64.sh

} > /dev/null 2>&1

echo "Generating makefiles with cmake"
{
    cmake-3.8.1-Linux-x86_64/bin/cmake ..
}

echo "Running make"
make -j4
make install

echo "Done"
