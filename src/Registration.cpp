#include <iostream>
#include "../include/Registration.h"
#include <pcl/registration/icp.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <vector>



int Registration::main() {
    return 0;
}


/**
 *
 */
CloudPtr
Registration::compute(std::vector<CloudPtr> input_pclouds){
    if(Registration::register_point_clouds(input_pclouds[0],input_pclouds[1])){
        return nullptr;
    }
    return nullptr;
}
/**
 * Runs the ICP algorithm to register the given pointclouds.
 *
 * @return True if ICP converges false otherwise
 */
bool
Registration::register_point_clouds(CloudPtr source_cloud, CloudPtr target_cloud) {

    // Start ICP
    pcl::IterativeClosestPoint<Point, Point> icp;
    CloudPtr target_cloud_new(new Cloud);
    CloudPtr final_cloud(new Cloud);

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

    *final_cloud = *source_cloud + *target_cloud;

    pcl::io::savePCDFileASCII("ICP_result.pcd", *final_cloud);

    return false;

}