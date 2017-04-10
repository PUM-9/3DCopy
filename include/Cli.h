//
// Created by hampus on 2017-04-04.
//

#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>

#ifndef INC_3DCOPY_CLI_H
#define INC_3DCOPY_CLI_H

namespace fs = boost::filesystem;

class Cli {
    public:
        // Methods
        Cli();
        int main(int argc, char* argv[]);

    private:
        // Fields
        std::vector<fs::path> sources = std::vector<fs::path>();
        std::string output_filename;
        bool mesh_only = false;
        bool register_only = false;
        bool verbose = false;

        // Methods
        int parse_arguments(int argc, char* argv[]);
        int parse_option(std::string option);
        int read_dir(fs::path path);
        void print_input();
        void add_source(fs::path path);
        pcl::PointCloud<pcl::PointXYZ>::Ptr register_point_clouds();
        void save_point_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);
        void save_mesh(const pcl::PolygonMesh polygon_mesh);
};


#endif //INC_3DCOPY_CLI_H
