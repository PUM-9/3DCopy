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
 * Launches Cura
 * @param mesh_file The filename of the .stl-file with the mesh to open
 * @return Returns the exit code of the Cura process
 */
int Slicer::launch_cura(std::string mesh_file) const {
    // Add the path to Cura to the PYTHONPATH environment variable so python can find it
    std::string python_path = "PYTHONPATH=" + cura_path;
    putenv(const_cast<char*>(python_path.c_str()));

    return system("python -m Cura.cura test.stl");
}

/**
 * Launches Slic3r
 * @param mesh_file The filename of the .stl-file with the mesh to open
 * @return Returns the exit code of the Slic3r process
 */
int Slicer::launch_slic3r(std::string mesh_file) const {
    std::string command = "slic3r --gui " + mesh_file;
    return system(command.c_str());
}
