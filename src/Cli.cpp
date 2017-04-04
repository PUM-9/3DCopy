//
// Created by hampus on 2017-04-04.
//

#include "../include/Cli.h"
#include <iostream>

int Cli::main(int argc, char **argv) {
    if (parse_arguments(argc, argv)) {
        std::cout << "Usage: " << argv[0] << " [options] sources output_filename" << std::endl;
        return 0;
    }

    return 0;
}

int Cli::parse_arguments(int argc, char **argv) {

    if (argc < 3) {
        return 1;
    }

}