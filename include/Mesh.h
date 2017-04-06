//
// Created by olof on 4/4/17.
//

#ifndef INC_3DCOPY_MESH_H
#define INC_3DCOPY_MESH_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::Normal> NormalCloud;

class Mesh {
    public:
        pcl::PolygonMesh mesh(const PointCloud::Ptr point_cloud);

    private:
        void poisson_reconstruction(pcl::PointCloud<pcl::PointNormal>::Ptr point_cloud, pcl::PolygonMesh& mesh);
        NormalCloud::Ptr estimate_normals(const PointCloud::Ptr point_cloud);
};

#endif //INC_3DCOPY_MESH_H
