//
// Created by hampus on 2017-04-04.
//

#include "../include/Cli.h"
#include "../include/Mesh.h"
#include "../include/Registration.h"
#include <pcl/io/vtk_lib_io.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
namespace fs = boost::filesystem;

/**
 *  Default constructor that initializes a few private values.
 */
Cli::Cli() {
    mesh_only = false;
    register_only = false;
    verbose = false;
    sources = std::vector<fs::path>();
}

/**
 * The main method that takes in the arguments and runs the program. Right now it only prints the inputed arguments but it's here
 * the execution of the rest of the program should be implemented.
 * @param argc the number of arguments
 * @param argv list of arguments
 * @return exit_code 0 if everything went ok otherwise non-zero.
 */
int Cli::main(int argc, char **argv) {

    Mesh mesh = Mesh();

    if (parse_arguments(argc, argv)) {
        std::cout << "Usage: " << argv[0] << " [options] source1:source2... output_filename" << std::endl;
        std::cout << "source are the .pcd files to be registered and output_filename is the filename of the output "
                "files." << std::endl;
        std::cout << "-m        just mesh the point cloud (only meshes the first point cloud)." << std::endl;
        std::cout << "-r        just registers the given point clouds, skips meshing." << std::endl;
        return 0;
    }

    if (mesh_only && !sources.empty()) {
        PointCloud::Ptr point_cloud_ptr (new PointCloud);
        pcl::io::loadPCDFile(sources.at(0).string(), *point_cloud_ptr);
        pcl::PolygonMesh polygon_mesh = mesh.mesh(point_cloud_ptr);
        save_mesh(polygon_mesh);
        return 0;
    }

    PointCloud::Ptr point_cloud = register_point_clouds();
    save_point_cloud(point_cloud);

    if (!register_only) {
        pcl::PolygonMesh polygon_mesh = mesh.mesh(point_cloud);
        save_mesh(polygon_mesh);
    }

    return 0;
}

/**
 * Parses option and sets internal fields accordingly.
 * @param option The option to parse
 * @return error code 0 if everything went ok otherwise non-zero.
 */
int Cli::parse_option(std::string option) {

    if (option == "-m") {
        mesh_only = true;
        register_only = false;
    } else if (option == "-r") {
        register_only = true;
        mesh_only = false;
    } else if (option == "-v") {
        verbose = true;
    }
    else {
        return 1;
    }

    return 0;

}

/**
 * Reads in all the files from a directory and if they are pcd files they are added to sources.
 * @param dir path to the directory.
 * @return error code 0 if everything went ok non-zero otherwise.
 */
int Cli::read_dir(fs::path path) {

    if (!fs::exists(path)) {
        std::cout << path << " not found." << std::endl;
        return 1;
    }

    try {
        if (fs::is_directory(path)) {
            fs::directory_iterator it = fs::directory_iterator(path);
            fs::directory_iterator end;
            for (it; it != end; ++it) {
                if (fs::is_regular_file(it->path())) {
                    add_source(it->path());
                }
            }

        }
    } catch (const fs::filesystem_error& ex) {
        std::cout << ex.what() << std::endl;
        return 1;
    }

    return 0;
}

/**
 * Parse the input from the command line and saving it to local fields.
 * @param argc the number of arguments
 * @param argv list of arguments
 * @return exit_code 0 if everything went ok otherwise non-zero.
 */
int Cli::parse_arguments(int argc, char **argv) {

    // Make sure there is enough arguments
    if (argc < 3) {
        std::cout << "To few arguments" << std::endl;
        return 1;
    }

    int counter = 1;
    int last = argc-1;
    std::string argument = std::string(argv[counter]);

    // Read options
    while (!argument.empty() && argument.at(0) == '-') {
        if (parse_option(argument)) {
            std::cout << "Unrecognized option: " << argument << std::endl;
            return 2;
        }
        counter++;
        argument = std::string(argv[counter]);
    }


    while (counter < last) {

        fs::path p = fs::path(argument);

        if (fs::exists(p)) {

            if (fs::is_directory(p)) {
                read_dir(p);
            } else if (fs::is_regular_file(p)) {
                add_source(p);
            }

        } else {
            std::cout << p << " not found." << std::endl;
        }

        counter++;
        argument = std::string(argv[counter]);
    }

    if (sources.empty()) {
        std::cout << "No sources found" << std::endl;
        return 4;
    }

    output_filename = std::string(argv[last]);

    if (verbose) {
        print_input();
    }

    return 0;

}

/**
 * Prints the local variables read in from the command line used for debugging and testing.
 */
void Cli::print_input() {
    std::cout << "Read sources:";
    for (size_t i=0; i < sources.size(); i++) {
        std::cout << " " << sources.at(i);
    }
    std::cout << std::endl;
    std::cout << "Output filename: " << output_filename << std::endl;
}

/**
 * Adds a file to sources after it has made sure the file exists and is a pcd file.
 * @param path The file path to be added to sources.
 */
void Cli::add_source(fs::path path) {
    if (fs::exists(path)) {
        if (fs::extension(path) == ".pcd") {
            sources.push_back(path);
        } else {
            std::cout << path << " not recognized as pcd file." << std::endl;
        }
    } else {
        std::cout << path << " not found." << std::endl;
    }
}

/**
 * Registers the point clouds in sources using the Registration class.
 * @return The resulting point cloud.
 */
PointCloud::Ptr Cli::register_point_clouds() {
    Registration registration = Registration();
    std::vector<PointCloud::Ptr> point_clouds;

    for (auto it=sources.begin(); it!=sources.end(); ++it) {
        PointCloud::Ptr point_cloud_ptr (new PointCloud);
        pcl::io::loadPCDFile((*it).string(), *point_cloud_ptr);
        point_clouds.push_back(point_cloud_ptr);
    }
    std::cout << "Read point clouds" << std::endl;
    return registration.register_point_clouds(point_clouds);
}

/**
 * Saves the point cloud to a .pcd file accordingly to the input from the user.
 * @param point_cloud The point cloud to be saved.
 */
void Cli::save_point_cloud(const PointCloud::Ptr point_cloud) {
    std::stringstream ss;
    ss << output_filename << ".pcd";
    pcl::io::savePCDFile(ss.str(), *point_cloud);
    std::cout << "Saved point cloud to " << ss.str() << std::endl;
}

/**
 * Saves a Polygon Mesh to .stl file accordingly to the input from the user.
 * @param polygon_mesh The Polygon Mesh to be saved.
 */
void Cli::save_mesh(const pcl::PolygonMesh polygon_mesh) {
    std::stringstream ss;
    ss << output_filename << ".stl";
    pcl::io::savePolygonFileSTL(ss.str(), polygon_mesh);
    std::cout << "Saved mesh to " << ss.str() << std::endl;
}