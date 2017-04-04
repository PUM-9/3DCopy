//
// A client that calls the mesh service to test it
// Needs the files lamppost.pcd, ism_test_cat.pcd and ism_test_wolf.pcd
// to send to the meshing service
//
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/surface/poisson.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::Normal> NormalCloud;

/**
 * Estimates the normals for a point cloud using PCL
 *
 * @param point_cloud The input point cloud that the normals are generated for
 * @return Returns a NormalCloud::Ptr with the generated normals
 */
NormalCloud::Ptr estimate_normals(PointCloud::Ptr point_cloud)
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
void poisson_reconstruction(pcl::PointCloud<pcl::PointNormal>::Ptr point_cloud, pcl::PolygonMesh& mesh)
{

    std::cout << "Begin poisson surface reconstruction" << std::endl;
    // Initialize poisson reconstruction
    pcl::Poisson<pcl::PointNormal> poisson;

    /*
     * Set the maximum depth of the tree used in Poisson surface reconstruction.
     * A higher value means more iterations which could lead to better results but
     * it is also more computaionally heavy.
     */
    poisson.setDepth(9);

    poisson.setInputCloud(point_cloud);

    // Perform the Poisson surface reconstruction algorithm
    poisson.reconstruct(mesh);
}

int main(int argc, char **argv)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lamp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cat(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_wolf(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCLPointCloud2 cloud_blob;
    if (pcl::io::loadPCDFile("lamppost.pcd", cloud_blob) == -1) {
        std::cout << "Couldn't read file bun0.pcd" << std::endl;
    }
    pcl::fromPCLPointCloud2(cloud_blob, *cloud_lamp);

    if (pcl::io::loadPCDFile("ism_test_cat.pcd", cloud_blob) == -1) {
        std::cout << "Couldn't read file bun0.pcd" << std::endl;
    }
    //pcl::fromPCLPointCloud2(cloud_blob, *cloud_cat);

    if (pcl::io::loadPCDFile("ism_test_wolf.pcd", cloud_blob) == -1) {
        std::cout << "Couldn't read file bun0.pcd" << std::endl;
    }
    //pcl::fromPCLPointCloud2(cloud_blob, *cloud_wolf);

    NormalCloud::Ptr normals = estimate_normals(cloud_lamp);

    // Add the normals to the point cloud
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud_lamp, *normals, *cloud_with_normals);

    pcl::PolygonMesh mesh;
    poisson_reconstruction(cloud_with_normals, mesh);

    std::stringstream file_name;
    file_name << "lampmesh.stl";

    if (pcl::io::savePolygonFile(file_name.str(), mesh)) {
        std::cout << "Saved file mesh-responseX.stl" << std::endl;
    }

    return 0;
}
