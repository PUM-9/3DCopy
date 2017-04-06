#ifndef INC_3DCOPY_REGISTRATION_H
#define INC_3DCOPY_REGISTRATION_H
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> Cloud;
typedef Cloud::Ptr CloudPtr;

class Registration {
public:
    int main();
    CloudPtr compute(std::vector<CloudPtr> input_pclouds);

private:
    bool icp_converged;

    CloudPtr register_point_clouds(CloudPtr target_cloud, CloudPtr source_cloud);
    bool has_converged();
};


#endif //INC_3DCOPY_REGISTRATION_H
