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
        std::cout << "Usage: " << argv[0] << " source1:source2... output_filename" << std::endl;
        std::cout << "source are the .pcd files to be registered and output_filename is the filename of the output "
                "files." << std::endl;
        return 0;
    }

    print_input();

    return 0;
}

int Cli::parse_arguments(int argc, char **argv) {

    // Make sure there is enough arguments
    if (argc < 4) {
        std::cout << "To few arguments" << std::endl;
        return 1;
    }

    int counter = 1;
    int last = argc-1;
    std::string argument = std::string(argv[counter]);

    //Read sources
    while (counter < last) {
        // Make sure it's a .pcd file
        if (is_pcd_file(argument)) {
            std::cout << "All input files must be .pcd files" << std::endl;
            return 3;
        }
        sources.push_back(argument);
        counter++;
        argument = std::string(argv[counter]);
    }

    output_filename = std::string(argv[last]);

    return 0;

}

void Cli::print_input() {
    std::cout << "Sources:";
    for (size_t i=0; i < sources.size(); i++) {
        std::cout << " " << sources.at(i);
    }
    std::cout << std::endl;
    std::cout << "Output filename: " << output_filename << std::endl;
}

bool Cli::is_pcd_file(std::string filename) {
    return filename.substr(filename.find_last_of(".") + 1) != "pcd";
}