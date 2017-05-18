#include <iostream>
#include "../include/Registration.h"
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <boost/log/trivial.hpp>


/**
 *  Registers all the pointclouds in the input vector
 *  @param input_pclouds Vector of point cloud pointers,
 *  @return Returns a pointer to the fully registered point cloud, returns a nullptr if given incorrect input
 */
Cloud::Ptr
Registration::register_point_clouds(std::vector<Cloud::Ptr> input_pclouds){
    BOOST_LOG_TRIVIAL(info) << "Registration started";
    if(input_pclouds.empty()) {
        return nullptr;
    }
    Cloud::Ptr final_cloud(new Cloud);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    unsigned int point_clouds = 0;
    voxel_filter.setLeafSize(leaf_size,leaf_size,leaf_size);
    for(Cloud::Ptr cloud : input_pclouds){
        if (verbose) {std::cout << "---------------------------------------------" << std::endl;}
        Eigen::Matrix4f transform = get_transform_to_target(final_cloud, cloud);
        if (verbose) {std::cout << "Registered point cloud #" << point_clouds++ << "." << std::endl;}
        Cloud::Ptr temp(new Cloud);
        *temp = *final_cloud;
        merge_clouds(temp, cloud, transform);
        voxel_filter.setInputCloud(temp);
        if(has_converged()){
            if(verbose) {std::cout << "Points in point cloud before filtering: " << temp->width*temp->height << std::endl;}
            voxel_filter.filter(*final_cloud);
            if(verbose) {std::cout << "Points in point cloud after filtering: " << final_cloud->width*final_cloud->height << std::endl;}

        } else {
            if(verbose) {std::cout << "ICP did not converge" << std::endl;}
        }
    }
    if(verbose) {std::cout << "---------------------------------------------" << std::endl;}
    return final_cloud;
}
/**
 * Runs the ICP algorithm to register the given pointclouds.
 * @param target_cloud Target cloud, will not be moved
 * @param source_cloud Source cloud, will be moved to match target
 * @return Returns a pointer to the final cloud
 */
Eigen::Matrix4f
Registration::get_transform_to_target(Cloud::Ptr target_cloud, Cloud::Ptr source_cloud) {

    // Start ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    Cloud::Ptr source_cloud_transformed(new Cloud);

    // Parameters for the ICP algorithm
    icp.setInputTarget(target_cloud);
    icp.setInputSource(source_cloud);
    icp.setMaximumIterations(this->max_iterations);
    icp.setTransformationEpsilon(this->transformation_epsilon);
    icp.setMaxCorrespondenceDistance(this->max_correspondence_distance);
    Eigen::Matrix4f transformation_matrix = Eigen::Matrix<float,4,4>::Identity ();
    icp.align(*target_cloud);
    if(verbose) {std::cout << "ICP ran for " << icp.nr_iterations_ << '/' << max_iterations << " iterations." << std::endl;}

    if (icp.hasConverged()) {
        if(verbose) {std::cout << "The score is " << icp.getFitnessScore() << std::endl;}
        transformation_matrix = icp.getFinalTransformation();
        if(verbose) {
            std::cout << "ICP converged/success." << std::endl;
            std::cout << "Transformation matrix:" << std::endl;
            std::cout << transformation_matrix << std::endl;
        }
        icp_converged = true;
    } else {
        if(verbose) {
            std::cout << "ICP did not converge./FAILED" << std::endl;
        }
        icp_converged = false;
    }
    pcl::transformPointCloud(*source_cloud, *source_cloud_transformed, transformation_matrix);
    pcl::io::savePCDFileASCII("ICP_result.pcd", *target_cloud + *source_cloud_transformed);
    return transformation_matrix;
}

/**
 * Merges new_cloud transformed to the target cloud
 * @param target Will have new_cloud added after function returns
 * @param new_cloud Will be transformed by transform and added to target
 * @param transform The transform to apply to new_cloud
 * @return void
 */
void
Registration::merge_clouds(Cloud::Ptr target, Cloud::Ptr new_cloud, Eigen::Matrix4f transform){
    pcl::transformPointCloud(*new_cloud,*new_cloud,transform);
    *target = *target + *new_cloud;
    return;
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