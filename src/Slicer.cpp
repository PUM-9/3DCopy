//
// Created by martin on 2017-05-03.
//

#include "../include/Slicer.h"

/**
 * Convenience function for checking if slicing is enabled (if boost version >= 1.64 was
 * found by cmake).
 * @return true if slicing is possible, false otherwise
 */
bool Slicer::is_enabled() {
    #ifdef ENABLE_SLICER
        return true;
    #endif
    return false;
}

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
    #ifdef ENABLE_SLICER
        if (slicer == Cura) {
            return launch_cura(mesh_file);
        } else if (slicer == Slic3r) {
            return launch_slic3r(mesh_file);
        }
    #endif //ENABLE_SLICER
    return 1;
}

/**
 * Launches Cura as a child process.
 * @param mesh_file The filename of the .stl-file with the mesh to open
 * @return Returns the exit code of the Cura process
 */
int Slicer::launch_cura(std::string mesh_file) const {
    #ifdef ENABLE_SLICER
        // Run Cura in a child process with environment variable PYTHONPATH
        // set correctly to find the python module
        std::error_code ec;
        bp::child child(bp::search_path("python"), "-m", "Cura.cura", mesh_file
                , bp::env["PYTHONPATH"] += {":" + cura_path}, ec);

        // An error occurred when starting child process
        if (ec.value()) {
            return ec.value();
        }

        // Wait for process to exit and return exit code
        child.wait();
        return child.exit_code();
    #endif //ENABLE_SLICER
    // Slicing is not enabled, return 1
    return 1;
}

/**
 * Launches Slic3r as a child process
 * @param mesh_file The filename of the .stl-file with the mesh to open
 * @return Returns the exit code of the Slic3r process
 */
int Slicer::launch_slic3r(std::string mesh_file) const {
    #ifdef ENABLE_SLICER
        // Run Slic3r in a child process
        std::error_code ec;
        bp::child child(bp::search_path("slic3r"), "--gui", mesh_file, ec);

        // An error occurred when starting child process
        if (ec.value()) {
            return ec.value();
        }

        // Wait for process to exit and return exit code
        child.wait();
        return child.exit_code();
    #endif //ENABLE_SLICER
    // Slicing is not enabled, return 1
    return 1;
}
