#include "../include/Mesh.h"
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/poisson.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::Normal> NormalCloud;

/**
 * Estimates the normals for a point cloud using PCL
 *
 * @param point_cloud The input point cloud that the normals are generated for
 * @return Returns a NormalCloud::Ptr with the generated normals
 */
NormalCloud::Ptr
Mesh::estimate_normals(const PointCloud::Ptr point_cloud)
{
    std::cout << "Estimating normals" << std::endl;

    // Declare PCL objects needed to perform normal estimation
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimation;
    NormalCloud::Ptr normals(new NormalCloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZ>);

    // Set input parameters for normal estimation
    search_tree->setInputCloud(point_cloud);
    normal_estimation.setInputCloud(point_cloud);
    normal_estimation.setSearchMethod(search_tree);

    /*
     * When estimating normals, the algorithm looks at the nearest neighbors of every point
     * and fits a plane to these points as close as it can. The normal of this plane is
     * the estimated normal of the point.
     * This sets how many of the nearest neighbors to look at when estimating normals.
     * Is a rough setting for accuracy that can be adjusted.
     * A lower number here means that corners in the point cloud will be more accurate,
     * too low a number will cause problems.
     */
    normal_estimation.setKSearch(20);

    // Perform normal estimation algorithm
    normal_estimation.compute(*normals);

    // Reverse the direction of all normals so that the face of the object points outwards.
    // Should not be necessary but it is easier when visualising the object in MeshLab etc.
    for (size_t i = 0; i < normals->size(); ++i) {
        normals->points[i].normal_x *= -1;
        normals->points[i].normal_y *= -1;
        normals->points[i].normal_z *= -1;
    }

    return normals;
}

/**
 * Uses poisson surface reconstruction algorithm to generate a 3D-mesh from a point cloud
 * with normals
 *
 * @param point_cloud The point cloud (including normals) that is used to generate the mesh
 * @param mesh The resulting 3D-mesh is saved to this reference so that it can be used from
 *             outside the function. This avoids having to return a copy of a PolygonMesh
 *             object which could be very big.
 */
void
Mesh::poisson_reconstruction(pcl::PointCloud<pcl::PointNormal>::Ptr point_cloud, pcl::PolygonMesh& mesh)
{

    int depth = 11;
    int solver_divide = 7;
    int iso_divide = 10;
    int samples_per_node = 1;
    float point_weight = 4.0f;

    std::cout << "Begin poisson surface reconstruction" << std::endl;
    // Initialize poisson reconstruction
    pcl::Poisson<pcl::PointNormal> poisson;

    /*
    * Set the maximum depth of the tree used in Poisson surface reconstruction.
    * A higher value means more iterations which could lead to better results but
    * it is also more computationally heavy.
    */
    poisson.setDepth(depth);
    poisson.setSolverDivide(solver_divide);
    poisson.setPointWeight(point_weight);
    poisson.setSamplesPerNode(samples_per_node);
    poisson.setInputCloud(point_cloud);

    // Perform the Poisson surface reconstruction algorithm
    poisson.reconstruct(mesh);
}

/**
 * Reconstruct a point cloud to a mesh by estimate the normals of the point cloud
 *
 * @param point_cloud The input point cloud that will be reconstructed
 * @return Returns a reconstructed mesh
 */
pcl::PolygonMesh
Mesh::mesh(const PointCloud::Ptr point_cloud)
{

    // Estimate the normals of the point cloud
    NormalCloud::Ptr normals = estimate_normals(point_cloud);

    // Add the normals to the point cloud
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*point_cloud, *normals, *cloud_with_normals);

    // Point cloud to mesh reconstruction
    pcl::PolygonMesh mesh;
    poisson_reconstruction(cloud_with_normals, mesh);



    return mesh;
}
