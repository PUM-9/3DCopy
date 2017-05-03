//
// Created by martin on 2017-05-03.
//

#ifndef INC_3DCOPY_SLICER_H
#define INC_3DCOPY_SLICER_H

#include <string>

#ifdef ENABLE_SLICER
#include "boost/process.hpp"
namespace bp = boost::process;
#endif //ENABLE_SLICER

enum Slicers {
    Cura, Slic3r
};

class Slicer {
    public:
        static bool enabled();
        int launch(Slicers slicer, std::string mesh_file) const;

        std::string get_cura_path() const { return cura_path; };
        void set_cura_path(std::string cura_path) { this->cura_path = cura_path; };

    private:
        std::string cura_path = "/usr/share/cura";

        int launch_cura(std::string mesh_file) const;
        int launch_slic3r(std::string mesh_file) const;
};

#endif //INC_3DCOPY_SLICER_H
