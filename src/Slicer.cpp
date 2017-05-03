//
// Created by martin on 2017-05-03.
//

#include <iostream>
#include "../include/Slicer.h"

void Slicer::launch(Slicers slicer, std::string mesh_file) {
    if (slicer == Cura) {
        // Run Cura in a child process with environment variable PYTHONPATH
        // set correctly
        bp::child child(bp::search_path("python"), "-m", "Cura.cura", mesh_file
                        , bp::env["PYTHONPATH"] += {":/usr/share/cura"});

        child.wait();

        int result = child.exit_code();
        std::cout << result << std::endl;
    } else if (slicer == Slic3r) {
        // Run Slic3r in a child process
        bp::child child(bp::search_path("slic3r"), "--gui", mesh_file);

        child.wait();

        int result = child.exit_code();
        std::cout << result << std::endl;
    }
}