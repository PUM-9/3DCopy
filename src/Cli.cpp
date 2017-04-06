//
// Created by hampus on 2017-04-04.
//

#include "../include/Cli.h"
#include "../include/Mesh.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

/**
 *  Default constructor that initializes a few private values.
 */
Cli::Cli() {
    source_is_dir = false;
    mesh_only = false;
    sources = std::vector<Path>();
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
        std::cout << "-d        source is a directory with the .pcd files." << std::endl;
        std::cout << "-m        just mesh the point cloud (only meshes the first point cloud)." << std::endl;
        return 0;
    }

    if (mesh_only && !sources.empty()) {
        PointCloud::Ptr point_cloud_ptr (new PointCloud);
        pcl::io::loadPCDFile(sources.at(0).string(), *point_cloud_ptr);
        pcl::PolygonMesh polygon_mesh = mesh.mesh(point_cloud_ptr);
        std::stringstream ss;
        ss << output_filename << ".stl";
        pcl::io::savePolygonFileSTL(ss.str(), polygon_mesh);
        std::cout << "Saved mesh to " << ss.str() << std::endl;
        return 0;
    }

    print_input();

    return 0;
}

/**
 * Parses option and sets internal fields accordingly.
 * @param option The option to parse
 * @return error code 0 if everything went ok otherwise non-zero.
 */
int Cli::parse_option(std::string option) {

    if (option == "-d") {
        source_is_dir = true;
    } else if (option == "-m") {
        mesh_only = true;
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
int Cli::read_dir(std::string dir) {

    Path path = Path(dir);

    try {
        if (boost::filesystem::exists(path)) {
            if (boost::filesystem::is_regular_file(path)) {
                sources.push_back(path);
                return 0;
            } else if (boost::filesystem::is_directory(path)) {

                boost::filesystem::directory_iterator it = boost::filesystem::directory_iterator(path);
                boost::filesystem::directory_iterator end;
                for (it; it != end; ++it) {
                    if (is_pcd_file(it->path().string())) {
                        sources.push_back(it->path());
                    } else {
                        std::cout << it->path() << " is not a pcd file." << std::endl;
                    }
                }

            }
        }
    } catch (const boost::filesystem::filesystem_error& ex) {
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
    if (argc < 4) {
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

    //Read sources
    if (source_is_dir) {
        read_dir(argument);
    } else {
        while (counter < last) {
            // Make sure it's a .pcd file
            if (!is_pcd_file(argument)) {
                std::cout << "All input files must be .pcd files" << std::endl;
                return 3;
            }
            Path path = Path(argument);
            if (boost::filesystem::exists(path)) {
                sources.push_back(path);
            } else {
                std::cout << "File not found: " << path << std::endl;
            }
            counter++;
            argument = std::string(argv[counter]);
        }
    }

    if (sources.empty()) {
        std::cout << "No sources found" << std::endl;
        return 4;
    }

    output_filename = std::string(argv[last]);

    return 0;

}

/**
 * Prints the local variables read in from the command line used for debugging and testing.
 */
void Cli::print_input() {
    std::cout << "Source is directory: " << source_is_dir << std::endl;
    std::cout << "Sources:";
    for (size_t i=0; i < sources.size(); i++) {
        std::cout << " " << sources.at(i);
    }
    std::cout << std::endl;
    std::cout << "Output filename: " << output_filename << std::endl;
}

/**
 * Checks if a file is a pcd file by checking if the file ending is '.pcd'.
 * @param filename The file to be checked.
 * @return True if it is a pcd file false otherwise.
 */
bool Cli::is_pcd_file(std::string filename) {
    return filename.substr(filename.find_last_of(".") + 1) == "pcd";
}
