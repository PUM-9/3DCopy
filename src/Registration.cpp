#include <iostream>
#include "../include/Registration.h"
#include <pcl/registration/icp.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/conversions.h>

int Registration::main() {
    return 0;
}

/**
 * Runs the ICP algorithm to register the given pointclouds.
 *
 * @return True if ICP converges false otherwise
 */
bool
Registration::register_point_clouds() {

    // Start ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    // Parameters for the ICP algorithm
    icp.setInputTarget(source_cloud);
    icp.setInputSource(target_cloud);
    icp.setMaximumIterations(25);
    icp.setTransformationEpsilon(1e-7);
    icp.setMaxCorrespondenceDistance(3);

    icp.align(*target_cloud);

    if (icp.hasConverged()) {
        std::cout << "ICP converged." << std::endl
                  << "The score is " << icp.getFitnessScore() << std::endl;
        std::cout << "Transformation matrix:" << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;
        Eigen::Matrix4f transformationMatrix = icp.getFinalTransformation();
        std::cout << "trans %n" << transformationMatrix << std::endl;

        pcl::transformPointCloud(*target_cloud, *target_cloud_new, transformationMatrix);

        *final_cloud = *source_cloud + *target_cloud;

        pcl::io::savePCDFileASCII("ICP_result.pcd", *final_cloud);
        return true;
    } else {
        std::cout << "ICP did not converge." << std::endl;
    }

    Eigen::Matrix4f transformationMatrix = icp.getFinalTransformation();
    std::cout << "trans %n" << transformationMatrix << std::endl;

    pcl::transformPointCloud(*target_cloud, *target_cloud_new, transformationMatrix);

    *final_cloud = *source.point_cloud_ptr + *target.point_cloud_ptr;

    pcl::io::savePCDFileASCII("ICP_result.pcd", *final_cloud);

    return false;

}