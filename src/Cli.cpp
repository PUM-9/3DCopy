//
// Created by hampus on 2017-04-04.
//

#include "../include/Cli.h"
#include <iostream>

Cli::Cli() {
    source_is_dir = false;
    sources = std::vector<std::string>();
}

int Cli::main(int argc, char **argv) {
    if (parse_arguments(argc, argv)) {
        std::cout << "Usage: " << argv[0] << " [options] source1:source2... output_filename" << std::endl;
        std::cout << "source are the .pcd files to be registered and output_filename is the filename of the output "
                "files." << std::endl;
        std::cout << "-d        source is a directory with the .pcd files." << std::endl;
        return 0;
    }

    std::cout << "Source is directory: " << source_is_dir << std::endl;
    std::cout << "Sources:";
    for (size_t i=0; i < sources.size(); i++) {
        std::cout << " " << sources.at(i);
    }
    std::cout << std::endl;
    std::cout << "Output filename: " << output_filename << std::endl;
    return 0;
}

int Cli::parse_option(std::string option) {

    if (option == "-d") {
        source_is_dir = true;
    } else {
        return 1;
    }

    return 0;

}

int Cli::read_dir(std::string path) {
    return 0;
}

int Cli::parse_arguments(int argc, char **argv) {

    // Make sure there is enough arguments
    if (argc < 3) {
        return 1;
    }

    int counter = 1;
    int last = argc-1;
    std::string argument = std::string(argv[counter]);

    // Read options
    while (!argument.empty() && argument.at(0) == '-') {
        if (parse_option(argument)) {
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
            if (argument.substr(argument.find_last_of(".") + 1) != "pcd") {
                return 3;
            }
            sources.push_back(argument);
            counter++;
            argument = std::string(argv[counter]);
        }
    }

    output_filename = std::string(argv[last]);

    return 0;

}