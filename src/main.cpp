#include <iostream>
#include "../include/Cli.h"

int main() {
    Cli cli;
    std::cout << "Hello, World! " << cli.main() <<  std::endl;
    return 0;
}