#ifndef INC_3DCOPY_REGISTRATION_H
#define INC_3DCOPY_REGISTRATION_H
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointCloud<pcl::PointXYZ> Cloud;

class Registration {
public:
    //Methods
    Cloud::Ptr register_point_clouds(std::vector<Cloud::Ptr> input_pclouds);
    void set_max_iterations(unsigned int iter);
    unsigned int get_max_iterations();
    void set_max_correspondence_distance(double distance);
    double get_max_correspondence_distance();
    void set_transformation_epsilon(double epsilon);
    double get_transformation_epsilon();
    void set_verbose_mode(bool mode);
    bool get_verbose_mode();
    void set_leaf_size(float size);
    float get_leaf_size();

private:
    //Data fields
    bool icp_converged;
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
    double max_correspondence_distance = 15;            //Maximum distance allowed between point a in cloud x and
                                                        // point a in cloud y
    unsigned int max_iterations = 100;                  //Force the ICP Algorithm to stop after max_iterations;
    double transformation_epsilon = 1e-7;               //How much ICP is allowed to move source in one iteration;
    bool verbose = false;
    float leaf_size = 0.43f;                            //Leaf size for the voxel filter

    //Methods
    Cloud::Ptr add_point_cloud_to_target(Cloud::Ptr target_cloud, Cloud::Ptr source_cloud);
    Cloud::Ptr register_point_clouds_small(std::vector<Cloud::Ptr> input_pclouds);
    bool has_converged();

};


#endif //INC_3DCOPY_REGISTRATION_H
