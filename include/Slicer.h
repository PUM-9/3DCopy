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
        int launch(Slicers slicer, std::string mesh_file) const;

    private:
        std::string cura_path = "/usr/share/cura";

        int launch_cura(std::string mesh_file) const;
        int launch_slic3r(std::string mesh_file) const;
};


#endif //INC_3DCOPY_SLICER_H
