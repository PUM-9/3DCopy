#include <iostream>
#include "../include/Registration.h"
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <vector>


/**
 *  Registers all the pointclouds in the input vector
 *  @param input_pclouds Vector of point cloud pointers,
 *  @return Returns a pointer to the fully registered point cloud, returns a nullptr if given incorrect input
 */
Cloud::Ptr
Registration::register_point_clouds(std::vector<Cloud::Ptr> input_pclouds){
    if(input_pclouds.size() > 6){
        std::vector<std::vector<Cloud::Ptr> > cloud_array;
        for(int i = 0; i < input_pclouds.size() ;i++){
            if(i%6 == 0){
                std::vector<Cloud::Ptr> temp;
                cloud_array.push_back(temp);
            }
            cloud_array[floor(i/6)].push_back(input_pclouds[i]);
        }
        std::vector<Cloud::Ptr> part_results;
        part_results.resize(cloud_array.size());
        for(int i = 0; i < cloud_array.size();i++){
            part_results[i] = register_point_clouds(cloud_array[i]);
        }
        std::vector<Eigen::Matrix4f> transformations;
        return part_results[0];
    } else {
        return register_point_clouds_small(input_pclouds);
    }
}

Cloud::Ptr
Registration::register_point_clouds_small(std::vector<Cloud::Ptr> input_pclouds){
    if (input_pclouds.empty()) {
        return nullptr;
    }
    Cloud::Ptr final_cloud = input_pclouds[0];
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    unsigned int registered_point_clouds = 0;
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    for (Cloud::Ptr cloud : input_pclouds) {
        std::cout << "---------------------------------------------" << std::endl;
        Cloud::Ptr temp = add_point_cloud_to_target(final_cloud, cloud);
        std::cout << "Registrered point cloud #" << registered_point_clouds++ << "." << std::endl;
        voxel_filter.setInputCloud(temp);
        if (has_converged()) {
            std::cout << "Points in point cloud before filtering: " << temp->width * temp->height << std::endl;
            voxel_filter.filter(*final_cloud);
            std::cout << "Points in point cloud after filtering: " << final_cloud->width * final_cloud->height
                      << std::endl;

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
Registration::set_leaf_size(float size){
    if (size > 0.0){
        this->leaf_size = size;
    }
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

float
Registration::get_leaf_size(){
    return this->leaf_size;
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