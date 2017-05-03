//
// Created by martin on 2017-05-03.
//

#ifndef INC_3DCOPY_SLICER_H
#define INC_3DCOPY_SLICER_H

#include "boost/process.hpp"

namespace bp = boost::process;

enum Slicers {
    Cura, Slic3r
};

class Slicer {
    public:
        void launch(Slicers slicer, std::string mesh_file);
};


#endif //INC_3DCOPY_SLICER_H
