//
// Created by olof on 4/5/17.
//

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::Normal> NormalCloud;

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

int main( int argc, char** argv )
{
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  if (pcl::io::loadPCDFile (argv[1], *cloud) < 0)
  {
    std::cout << "Error loading  cloud." << std::endl;
    return (-1);
  }

    *cloud_normals = estimate_normals(cloud);

  std::cout << "number of points in the cloud " << cloud->points.size() <<std::endl;
  std::cout << "number of normals in the cloud " << cloud_normals->points.size() <<std::endl;


  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addPointCloudNormals <pcl::PointXYZRGB, pcl::Normal> (cloud, cloud_normals, 30, 0.1, "normals", 0);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4, "normals");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "normals");



  while (!viewer->wasStopped ()) {
    viewer->spinOnce ();
  }

  return 0;
}