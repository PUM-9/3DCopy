//
// Created by martin on 2017-05-03.
//

#include "../include/Slicer.h"

int Slicer::launch(Slicers slicer, std::string mesh_file) const {
    if (slicer == Cura) {
        launch_cura(mesh_file);
    } else if (slicer == Slic3r) {
        launch_slic3r(mesh_file);
    }
}

int Slicer::launch_cura(std::string mesh_file) const {
    // Run Cura in a child process with environment variable PYTHONPATH
    // set correctly to find the python module
    bp::child child(bp::search_path("python"), "-m", "Cura.cura", mesh_file
            , bp::env["PYTHONPATH"] += {":/usr/share/cura"});

    child.wait();
    return child.exit_code();
}

int Slicer::launch_slic3r(std::string mesh_file) const {
    // Run Slic3r in a child process
    bp::child child(bp::search_path("slic3r"), "--gui", mesh_file);

    child.wait();
    return child.exit_code();
}