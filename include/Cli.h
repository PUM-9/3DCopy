//
// Created by hampus on 2017-04-04.
//

#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>

#ifndef INC_3DCOPY_CLI_H
#define INC_3DCOPY_CLI_H

typedef boost::filesystem::path Path;

class Cli {
    public:
        // Methods
        Cli();
        int main(int argc, char* argv[]);

    private:
        // Fields
        std::vector<Path> sources;
        std::string output_filename;
        bool source_is_dir;
        bool mesh_only;
        bool register_only;

        // Methods
        int parse_arguments(int argc, char* argv[]);
        int parse_option(std::string option);
        int read_dir(std::string path);
        void print_input();
        bool is_pcd_file(std::string filename);
        pcl::PointCloud<pcl::PointXYZ>::Ptr register_point_clouds();
        void save_point_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);
        void save_mesh(const pcl::PolygonMesh polygon_mesh);
};


#endif //INC_3DCOPY_CLI_H
