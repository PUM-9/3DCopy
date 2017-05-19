#!/bin/bash

echo "Installing dependecies"
{
    # Find which version of Ubuntu is running
    source /etc/lsb-release
    if [ DISTRIB_RELEASE == "14.04" ]; then 
	# Add PPA for installing PCL
	sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl -y
	sudo apt-get update

	# Install PCL
	sudo apt-get install libpcl-all -y
    else
	# Probably later version, just install PCL
	sudo apt-get update
	sudo apt-get install libpcl-dev
    fi

    # Install basic build dependencies and Boost	
    sudo apt-get install make g++ libboost-all-dev -y

} > /dev/null 2>&1

echo "Creating build dir"
mkdir -p build
cd build

echo "Downloading cmake version 3.8.1"
{
    # Download recent version of cmake
    wget https://cmake.org/files/v3.8/cmake-3.8.1-Linux-x86_64.sh
    chmod +x cmake-3.8.1-Linux-x86_64.sh
    ./cmake-3.8.1-Linux-x86_64.sh --include-subdir
    rm cmake-3.8.1-Linux-x86_64.sh

} > /dev/null 2>&1

echo "Generating makefiles"
{
    cmake-3.8.1-Linux-x86_64/bin/cmake ..
} > /dev/null 2>&1

echo "Building 3DCopy"
make -j4 > /dev/null 2>&1
sudo make install

echo "Done"
