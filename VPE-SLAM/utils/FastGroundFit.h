#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include "utils/common.h"
// using eigen lib
#include <Eigen/Dense>

/*
    @brief Ground Plane fitting ROS Node.
    @param Velodyne Pointcloud topic.
    @param Sensor Model.
    @param Sensor height for filtering error mirror points.
    @param Num of segment, iteration, LPR
    @param Threshold of seeds distance, and ground plane distance

    @subscirbe:/velodyne_points
    @publish:/points_no_ground, /points_ground
*/
class GroundPlaneFit{
public:
    //GroundPlaneFit(ros::NodeHandle &node_handle);
    GroundPlaneFit();

    std::vector<int> put_pointsCloud(pcl::PointCloud<PointType>::Ptr in_cloud);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    void fitlerGroundWithNormal(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_in, Eigen::Vector3d ref_vec, double th);
    void fitlerTopWithNormal(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_in, Eigen::Vector3d ref_vec, double th);

private:
    //ros::NodeHandle &node_handle_;
    int sensor_model_;
    double sensor_height_;
    int num_seg_;
    int num_iter_;
    int num_lpr_;
    double num_lpr_dig;
    double th_seeds_;
    double th_dist_;


    void estimate_plane_(void);
    void extract_initial_seeds_(const pcl::PointCloud<PointType>& p_sorted);
    // Model parameter for ground plane fitting
    // The ground plane model is: ax+by+cz+d=0
    // Here normal:=[a,b,c], d=d
    // th_dist_d_ = threshold_dist - d
    float d_;
    Eigen::MatrixXf normal_;
    float th_dist_d_;
};


