#include <iostream>
#include "../include/Registration.h"
#include <pcl/registration/icp.h>

#include <pcl/io/pcd_io.h>


int Registration::main() {
    return 0;
}


/**
 *
 */
CloudPtr
Registration::compute(std::vector<CloudPtr> input_pclouds){
    CloudPtr final_cloud;
    for(CloudPtr cloud : input_pclouds){
        CloudPtr temp = Registration::register_point_clouds(final_cloud,cloud);
        if(Registration::has_converged()){
            final_cloud = temp;
        } else {
            std::cout << "ICP did not converge" << std::endl;
            //throw new ICP_DID_NOT_CONVERGE_EXCEPTION;
        }
    }
    return final_cloud;
}
/**
 * Runs the ICP algorithm to register the given pointclouds.
 *
 * @return Returns a pointer to the final cloud
 */
CloudPtr
Registration::register_point_clouds(CloudPtr target_cloud, CloudPtr source_cloud) {

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
        icp_converged = true;
        return final_cloud;
    } else {
        std::cout << "ICP did not converge." << std::endl;
    }

    Eigen::Matrix4f transformationMatrix = icp.getFinalTransformation();
    std::cout << "trans %n" << transformationMatrix << std::endl;

    pcl::transformPointCloud(*target_cloud, *target_cloud_new, transformationMatrix);

    *final_cloud = *source_cloud + *target_cloud;

    pcl::io::savePCDFileASCII("ICP_result.pcd", *final_cloud);
    icp_converged = false;
    return final_cloud;

}

bool
Registration::has_converged(){
    return icp_converged;
}