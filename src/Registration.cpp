#include <iostream>
#include "../include/Registration.h"
#include <pcl/registration/icp.h>

#include <pcl/io/pcd_io.h>


/**
 *  Registers all the pointclouds in the input vector
 *  @param input_pclouds Vector of point cloud pointers,
 *  @return Returns a pointer to the fully registered point cloud
 */
Cloud::Ptr
Registration::register_point_clouds(std::vector<Cloud::Ptr> input_pclouds){
    Cloud::Ptr final_cloud(new Cloud);
    for(Cloud::Ptr cloud : input_pclouds){
        Cloud::Ptr temp = add_point_cloud_to_target(final_cloud, cloud);
        if(has_converged()){
            final_cloud = temp;
        } else {
            std::cout << "ICP did not converge" << std::endl;
        }
    }
    return final_cloud;
}
/**
 * Runs the ICP algorithm to register the given pointclouds.
 * @param target_cloud Target cloud, will not be moved
 * @param source_cloud Source cloud, will be moved to match target
 * @return Returns a pointer to the final cloud
 */
Cloud::Ptr
Registration::add_point_cloud_to_target(Cloud::Ptr target_cloud, Cloud::Ptr source_cloud) {

    // Start ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    Cloud::Ptr target_cloud_new(new Cloud);
    Cloud::Ptr final_cloud(new Cloud);

    // Parameters for the ICP algorithm
    icp.setInputTarget(source_cloud);
    icp.setInputSource(target_cloud);
    icp.setMaximumIterations(25);
    icp.setTransformationEpsilon(1e-7);
    icp.setMaxCorrespondenceDistance(3);

    icp.align(*target_cloud);

    if (icp.hasConverged()) {
        std::cout << "ICP converged/success." << std::endl
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
        std::cout << "ICP did not converge./FAILED" << std::endl;
        icp_converged = false;
    }

    Eigen::Matrix4f transformationMatrix = icp.getFinalTransformation();
    std::cout << "trans %n" << transformationMatrix << std::endl;

    pcl::transformPointCloud(*target_cloud, *target_cloud_new, transformationMatrix);

    *final_cloud = *source_cloud + *target_cloud;

    pcl::io::savePCDFileASCII("ICP_result.pcd", *final_cloud);
    return final_cloud;

}

bool
Registration::has_converged(){
    return icp_converged;
}