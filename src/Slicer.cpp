//
// Created by martin on 2017-05-03.
//

#include "../include/Slicer.h"

/**
 * Will run the specified slicer as a child process to the current process.
 * This function is blocking so it it will wait until the slicer is closed
 * before it returns.
 * @param slicer The slicer to use, either Cura or Slic3r
 * @param mesh_file The filename of the .stl-file with the mesh to open in the slicer
 * @return Returns the exit code of the child process (0 if successful exit), or
 *         1 if a valid slicer was not given
 */
int Slicer::launch(Slicers slicer, std::string mesh_file) const {
    if (slicer == Cura) {
        return launch_cura(mesh_file);
    } else if (slicer == Slic3r) {
        return launch_slic3r(mesh_file);
    }
    return 1;
}

/**
 * Launches Cura as a child process.
 * @param mesh_file The filename of the .stl-file with the mesh to open
 * @return Returns the exit code of the Cura process
 */
int Slicer::launch_cura(std::string mesh_file) const {
    // Run Cura in a child process with environment variable PYTHONPATH
    // set correctly to find the python module
    bp::child child(bp::search_path("python"), "-m", "Cura.cura", mesh_file
            , bp::env["PYTHONPATH"] += {":" + cura_path});

    // Wait for process to exit and return exit code
    child.wait();
    return child.exit_code();
}

/**
 * Launches Slic3r as a child process
 * @param mesh_file The filename of the .stl-file with the mesh to open
 * @return Returns the exit code of the Slic3r process
 */
int Slicer::launch_slic3r(std::string mesh_file) const {
    // Run Slic3r in a child process
    bp::child child(bp::search_path("slic3r"), "--gui", mesh_file);
    
    // Wait for process to exit and return exit code
    child.wait();
    return child.exit_code();
}