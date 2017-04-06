#ifndef INC_3DCOPY_REGISTRATION_H
#define INC_3DCOPY_REGISTRATION_H
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>

typedef pcl::PointCloud<pcl::PointXYZ> Cloud;

class Registration {
public:
    //Methods
    Cloud::Ptr register_point_clouds(std::vector<Cloud::Ptr> input_pclouds);

private:
    //Data fields
    bool icp_converged;

    //Methods
    Cloud::Ptr add_point_cloud_to_target(Cloud::Ptr target_cloud, Cloud::Ptr source_cloud);
    bool has_converged();
};


#endif //INC_3DCOPY_REGISTRATION_H
