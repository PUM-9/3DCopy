# 3DCopy

This program utilizes [PCL](https://www.pointclouds.org/ "Point Cloud Library") to register multiple point clouds into one complete point cloud of the whole object and then generate a mesh of the object. It was the result of a project in the course TDDD96 at Linköping University in 2017.

## Usage

The program provides a command line interface. Running it with the `-h` flag shows a usage:

```
Usage: 3DCopy [options] source1:source2... output_filename
source are the .pcd files to be registered and output_filename is the filename of the output files.
Allowed options:
  -h [ --help ]               produce help message.
  -v [ --verbose ]            run the program in verbose mode.
  -m [ --mesh-only ]          only mesh the pcd file.
  -r [ --register-only ]      only register the pcd files.
  -d [ --max-corr-dist ] arg  Maximum distance allowed between points in
                              different clouds. (>0)
  -i [ --max-iterations ] arg Maximum number of iterations ICP are allowed to
                              do. (>0)
  -l [ --log-level ] arg      Set the log level to: trace, debug, info
                              (default), warn, or error. Default
  --log-file arg              The filename of the log file. Default filename is
                              3DCopy_log.
```

## Installation

#### Install script
A script is provided for installing 3DCopy on a Ubuntu system. To install 3DCopy on Ubuntu, run these commands:

```
git clone https://github.com/PUM-9/3DCopy.git
cd 3DCopy
./install.sh
```

#### Build it yourself
If you're not using Ubuntu or if for some other reason the scipt doesn't work for you, you need to build it yourself. Its dependencies are [Boost](http://www.boost.org/ "Boost") and [PCL](https://www.pointclouds.org/ "Point Cloud Library"). You also need [CMake](http://www.cmake.org "CMake") and the usual dependencies for building C++. When all dependencies are met, run:

```
git clone https://github.com/PUM-9/3DCopy.git
cd 3DCopy
mkdir build
cd build
cmake ..
make
make install
```

## License

This program is licensed under the GNU Lesser General Public License 3.
