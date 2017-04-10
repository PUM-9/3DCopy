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
    void set_max_iterations(int iter);
    int get_max_iterations();
    void set_max_correspondence_distance(double distance);
    double get_max_correspondence_distance();
    void set_transformation_epsilon(double epsilon);
    double get_transformation_epsilon();
    void set_verbose_mode(bool mode);
    bool get_verbose_mode();
    void set_ransac_threshold(double threshold);
    double get_ransac_threshold();
    void set_euclidean_fitness(double epsilon);
    double get_euclidean_fitness();

private:
    //Data fields
    bool icp_converged;
    double max_correspondence_distance = 15;    //How far the input pclouds start out
    int max_iterations = 100;                   //Force the ICP Algorithm to stop after max_iterations;
    double transformation_epsilon = 1e-7;       //How much ICP is allowed to move source in one iteration;
    double euclidean_fitness = 1;               //Currently unused
    double ransac_rejection_threshold = 1;      //Currently unused
    bool verbose = false;


    //Methods
    Cloud::Ptr add_point_cloud_to_target(Cloud::Ptr target_cloud, Cloud::Ptr source_cloud);
    bool has_converged();

};


#endif //INC_3DCOPY_REGISTRATION_H
