//
// Created by hampus on 2017-04-04.
//

#include <string>
#include <vector>

#ifndef INC_3DCOPY_CLI_H
#define INC_3DCOPY_CLI_H


class Cli {
    public:
        Cli();
        int main(int argc, char* argv[]);

    private:
        std::vector<std::string> sources;
        std::string output_filename;
        bool source_is_dir;
        int parse_arguments(int argc, char* argv[]);
        void print_input();
        bool is_pcd_file(std::string filename);
};


#endif //INC_3DCOPY_CLI_H
