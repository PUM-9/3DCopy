//
// Created by Fredrik Wallström on 2017-04-05.
//

#ifndef INC_3DCOPY_NORMALS_H
#define INC_3DCOPY_NORMALS_H

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::Normal> NormalCloud;

class Normals {
    public:
        int main();

    private:
        NormalCloud::Ptr estimate_normals(pcl::PointCloud<pcl::PointXYZRGB> point_cloud);
};

#endif //INC_3DCOPY_NORMALS_H
