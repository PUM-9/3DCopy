//
// Created by hampus on 2017-04-04.
//

#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <boost/program_options.hpp>

#ifndef INC_3DCOPY_CLI_H
#define INC_3DCOPY_CLI_H

namespace fs = boost::filesystem;
namespace po = boost::program_options;

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
        void print_help(po::options_description options, char **argv);
        int parse_arguments(int argc, char* argv[]);
        int read_dir(fs::path path);
        void print_input();
        void add_source(fs::path path);
        pcl::PointCloud<pcl::PointXYZ>::Ptr register_point_clouds();
        void save_point_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);
        void save_mesh(const pcl::PolygonMesh polygon_mesh);
        void load_values(po::variables_map vm);
};


#endif //INC_3DCOPY_CLI_H
