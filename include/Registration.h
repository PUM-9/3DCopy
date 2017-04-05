#ifndef INC_3DCOPY_REGISTRATION_H
#define INC_3DCOPY_REGISTRATION_H
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> Cloud;
typedef Cloud::Ptr CloudPtr;

class Registration {
public:
    int main();
    CloudPtr compute(std::vector<CloudPtr> input_pclouds);

private:
    bool register_point_clouds(CloudPtr source_cloud, CloudPtr target_cloud);
};


#endif //INC_3DCOPY_REGISTRATION_H
