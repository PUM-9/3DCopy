//
// Created by hampus on 2017-04-04.
//

#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <boost/program_options.hpp>
#include <boost/log/trivial.hpp>

#ifndef INC_3DCOPY_CLI_H
#define INC_3DCOPY_CLI_H

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace logging = boost::log;

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
        double max_correspondence_distance = 15;    //Maximum distance allowed between point a in cloud x and
                                                    // point a in cloud y
        unsigned int max_iterations = 100;          //Force the ICP Algorithm to stop after max_iterations
        double transformation_epsilon = 1e-7;       //How much ICP is allowed to move source in one iteration
        logging::trivial::severity_level log_lvl = logging::trivial::info;
        std::string log_filename = "3DCopy_log";

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
        void init_logging();
};


#endif //INC_3DCOPY_CLI_H
