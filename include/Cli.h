//
// Created by hampus on 2017-04-04.
//

#include <string>

#ifndef INC_3DCOPY_CLI_H
#define INC_3DCOPY_CLI_H


class Cli {
    public:
        Cli();
        int main(int argc, char* argv[]);

    private:
        std::string source;
        std::string output_filename;
        bool source_is_dir;
        int parse_arguments(int argc, char* argv[]);
};


#endif //INC_3DCOPY_CLI_H
