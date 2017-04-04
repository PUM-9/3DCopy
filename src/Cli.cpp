//
// Created by hampus on 2017-04-04.
//

#include "../include/Cli.h"
#include <iostream>

Cli::Cli() {
    source_is_dir = false;
}

int Cli::main(int argc, char **argv) {
    if (parse_arguments(argc, argv)) {
        std::cout << "Usage: " << argv[0] << " [options] source1:source2... output_filename" << std::endl;
        std::cout << "source are the .pcd files to be registered and output_filename is the filename of the output "
                "files." << std::endl;
        std::cout << "-d        source is a directory with the .pcd files." << std::endl;
        return 0;
    }

    std::cout << source_is_dir << std::endl;

    return 0;
}

int Cli::parse_arguments(int argc, char **argv) {

    if (argc < 3) {
        return 1;
    }

    for (int i=0; i < argc; i++) {

        std::string argument = std::string(argv[i]);

        if (argument == "-d") {
            source_is_dir = true;
        }



    }

    return 0;

}