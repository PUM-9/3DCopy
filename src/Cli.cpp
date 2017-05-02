//
// Created by hampus on 2017-04-04.
//

#include "../include/Cli.h"
#include "../include/Mesh.h"
#include "../include/Registration.h"
#include <pcl/io/vtk_lib_io.h>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace logging = boost::log;
namespace keywords = boost::log::keywords;

/**
 *  Default constructor.
 */
Cli::Cli() {}

/**
 * The main method that takes in the arguments and runs the program. Right now it only prints the inputed arguments but it's here
 * the execution of the rest of the program should be implemented.
 * @param argc the number of arguments
 * @param argv list of arguments
 * @return exit_code 0 if everything went ok otherwise non-zero.
 */
int Cli::main(int argc, char **argv) {

    int exit_code = parse_arguments(argc, argv);
    init_logging();

    if (exit_code) {
        if (exit_code == 2) {// Displayed help
            return 0;
        }
        std::cout << "Failed to parse arguments, exit code: " << exit_code << std::endl;
        return 1;
    }

    Mesh mesh = Mesh();

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

    po::options_description options("Allowed options");
    options.add_options()
            ("help,h", "produce help message.")
            ("verbose,v", "run the program in verbose mode.")
            ("mesh-only,m", "only mesh the pcd file.")
            ("register-only,r", "only register the pcd files.")
            ("max-corr-dist,d", po::value<double>(), "Maximum distance allowed between points in different clouds. (>0)")
            ("max-iterations,i", po::value<int>(), "Maximum number of iterations ICP are allowed to do. (>0)");

    po::options_description filenames("Input and output files");
    filenames.add_options()
            ("filenames", po::value<std::vector<std::string> >(),
             "filenames of input files, directories and output filename");

    po::options_description all("All options");
    all.add(options).add(filenames);

    po::positional_options_description positional;
    positional.add("filenames", -1);

    po::variables_map vm;
    try {
        auto parser = po::command_line_parser(argc, (const char *const *) argv).options(all).positional(positional)
                .allow_unregistered().run();
        po::store(parser, vm);
        po::notify(vm);
    } catch (const po::error& e) {
        std::cerr << e.what() << std::endl;
        print_help(options, argv);
        return 1;
    }

    if (vm.count("help") || !vm.count("filenames")) {
        print_help(options, argv);
        return 2;
    }

    load_values(vm);

    if (sources.empty()) {
        std::cout << "No sources found" << std::endl;
        print_help(options, argv);
        return 3;
    }

    return 0;

}

/**
 * Loads values from the command line in to local fields that can be used later in the program.
 * @param vm variabel map with the values from the command line.
 */
void Cli::load_values(po::variables_map vm) {

    if (vm.count("verbose")) {
        verbose = true;
    }

    if (vm.count("mesh-only")) {
        mesh_only = true;
    }

    if (vm.count("register-only")) {
        register_only = true;
    }

    if (register_only && mesh_only) {
        register_only = mesh_only = false;
    }

    if (vm.count("max-corr-dist") && vm["max-corr-dist"].as<double>() > 0) {
        max_correspondence_distance = vm["max-corr-dist"].as<double>();
    }

    if (vm.count("max-iterations") && vm["max-iterations"].as<int>() > 0) {
        max_iterations = (unsigned int)vm["max-iterations"].as<int>();
    }

    if (vm.count("filenames")) {
        auto input_files = vm["filenames"].as<std::vector<std::string> >();
        size_t last = input_files.size() - 1;
        output_filename = input_files[last];
        for (size_t i=0; i < last; ++i) {
            fs::path p = fs::path(input_files[i]);
            if (fs::is_directory(p)) {
                read_dir(p);
            } else if (fs::is_regular_file(p)) {
                add_source(p);
            }
        }
    }

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
    std::cout << "mesh_only: " << mesh_only << std::endl;
    std::cout << "verbose: " << verbose << std::endl;
    std::cout << "register_only: " << register_only << std::endl;
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

    registration.set_verbose_mode(verbose);
    registration.set_max_correspondence_distance(max_correspondence_distance);
    registration.set_max_iterations(max_iterations);
    registration.set_transformation_epsilon(transformation_epsilon);

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
    BOOST_LOG_TRIVIAL(info) << "Saved point cloud to " << ss.str();
    if (verbose) {std::cout << "Saved point cloud to " << ss.str() << std::endl;}
}

/**
 * Saves a Polygon Mesh to .stl file accordingly to the input from the user.
 * @param polygon_mesh The Polygon Mesh to be saved.
 */
void Cli::save_mesh(const pcl::PolygonMesh polygon_mesh) {
    std::stringstream ss;
    ss << output_filename << ".stl";
    pcl::io::savePolygonFileSTL(ss.str(), polygon_mesh);
    BOOST_LOG_TRIVIAL(info) << "Saved mesh to " << ss.str();
    if (verbose){std::cout << "Saved mesh to " << ss.str() << std::endl;}
}

/**
 * Prints the help message.
 * @param options The options to be printed in the message.
 * @param argv List of arguments passed from the CLI used to get the command used.
 */
void Cli::print_help(po::options_description options, char **argv) {
    std::cout << "Usage: " << argv[0] << " [options] source1:source2... output_filename" << std::endl;
    std::cout << "source are the .pcd files to be registered and output_filename is the filename of the output "
            "files." << std::endl;
    std::cout << options << std::endl;
}


void Cli::init_logging() {
    fs::path log_path = "";
    logging::add_common_attributes();
    logging::add_file_log(
            keywords::file_name = "3DCopy_%N.log",
            keywords::rotation_size = 10 * 1024 * 1024,
            keywords::auto_flush = true,
            keywords::format = "[%TimeStamp%]: %Message%"
    );
    logging::core::get()->set_filter(logging::trivial::severity >= log_lvl);
    BOOST_LOG_TRIVIAL(info) << "Logging initialized";
}
