#include <iostream>
#include "../include/Registration.h"
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>


/**
 *  Registers all the pointclouds in the input vector
 *  @param input_pclouds Vector of point cloud pointers,
 *  @return Returns a pointer to the fully registered point cloud, returns a nullptr if given incorrect input
 */
Cloud::Ptr
Registration::register_point_clouds(std::vector<Cloud::Ptr> input_pclouds){
    if(input_pclouds.empty()) {
        return nullptr;
    }
    Cloud::Ptr final_cloud = input_pclouds[0];
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    unsigned int point_clouds = 0;
    voxel_filter.setLeafSize(leaf_size,leaf_size,leaf_size);
    //for (auto it = input_pclouds.begin()+1; it != input_pclouds.end(); ++it) {
    for(Cloud::Ptr cloud : input_pclouds){
        std::cout << "---------------------------------------------" << std::endl;
        Cloud::Ptr temp = add_point_cloud_to_target(final_cloud, cloud);
        std::cout << "Registrered point cloud #" << point_clouds++ << "." << std::endl;
        voxel_filter.setInputCloud(temp);
        if(has_converged()){
            std::cout << "Points in point cloud before filtering: " << temp->width*temp->height << std::endl;
            voxel_filter.filter(*final_cloud);      //final_cloud = temp, but with filtering
            //final_cloud = temp;
            std::cout << "Points in point cloud after filtering: " << final_cloud->width*final_cloud->height << std::endl;

        } else {
            std::cout << "ICP did not converge" << std::endl;
        }
    }
    std::cout << "---------------------------------------------" << std::endl;
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
    icp.setMaximumIterations(this->max_iterations);
    icp.setTransformationEpsilon(this->transformation_epsilon);
    icp.setMaxCorrespondenceDistance(this->max_correspondence_distance);

    icp.align(*target_cloud);
    std::cout << "ICP ran for " << icp.nr_iterations_ << " iterations" << std::endl;

    if (icp.hasConverged()) {
        std::cout << "The score is " << icp.getFitnessScore() << std::endl;
        if(verbose) {
            std::cout << "ICP converged/success." << std::endl;
            std::cout << "Transformation matrix:" << std::endl;
            std::cout << icp.getFinalTransformation() << std::endl;
        }
        icp_converged = true;
    } else {
        if(verbose) {
            std::cout << "ICP did not converge./FAILED" << std::endl;
        }
        icp_converged = false;
    }

    Eigen::Matrix4f transformationMatrix = icp.getFinalTransformation();
    if(verbose) {
        std::cout << "trans \n" << transformationMatrix << std::endl;
    }

    pcl::transformPointCloud(*target_cloud, *target_cloud_new, transformationMatrix);

    *final_cloud = *source_cloud + *target_cloud;

    pcl::io::savePCDFileASCII("ICP_result.pcd", *final_cloud);
    return final_cloud;

}

bool
Registration::has_converged(){
    return icp_converged;
}

void
Registration::set_max_correspondence_distance(double distance){
    if (distance > 0) {
        this->max_correspondence_distance = distance;
    }
}

void
Registration::set_max_iterations(unsigned int iter){
    this->max_iterations = iter;
}

void
Registration::set_transformation_epsilon(double epsilon){
    if (epsilon > 0) {
        this->transformation_epsilon = epsilon;
    }
}

unsigned int
Registration::get_max_iterations(){
    return this->max_iterations;
}

double
Registration::get_max_correspondence_distance(){
    return this->max_correspondence_distance;
}

double
Registration::get_transformation_epsilon(){
    return this->transformation_epsilon;
}

void
Registration::set_verbose_mode(bool mode){
    this->verbose = mode;
}

bool
Registration::get_verbose_mode(){
    return this->verbose;
}