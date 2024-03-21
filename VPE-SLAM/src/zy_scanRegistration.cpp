#include <cmath>
#include <vector>

#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <eigen3/Eigen/Core>
#include "utils/Entity_Cla.h"
#include "utils/FastGroundFit.h"
#include "utils/pcl_viewer.h"
#include <ctime>

typedef pcl::PointXYZINormal PointType;
clock_t begin_time;

ros::Publisher pubLaserCloud;
ros::Publisher pubCornerPointsSharp;
ros::Publisher pubSurfPointsFlat;
ros::Publisher pubEdgPoints;
ros::Publisher pubLaserCloud_temp;

ros::Publisher pubLaserCloud_zy;
ros::Publisher pubCornerPointsSharp_zy;
ros::Publisher pubSurfPointsFlat_zy;
pcl::PointCloud<PointType>::Ptr laserCloudCorner(new pcl::PointCloud<PointType>());
//surf feature
pcl::PointCloud<PointType>::Ptr laserCloudSurf(new pcl::PointCloud<PointType>());
//ful res
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
//
pcl::PointCloud<PointType>::Ptr laserCloudGround(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudFullRes_temp(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr cloud_in_temp(new pcl::PointCloud<PointType>());
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZINormal>);
pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_no_ground(new pcl::PointCloud<pcl::PointXYZINormal>);
pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_top(new pcl::PointCloud<pcl::PointXYZINormal>);
pcl::PointCloud<pcl::PointXYZINormal>::Ptr intespoints_used_test(new pcl::PointCloud<pcl::PointXYZINormal>);
pcl::PointCloud<pcl::PointXYZINormal>::Ptr edge_points(new pcl::PointCloud<pcl::PointXYZINormal>);
pcl::PointCloud<pcl::PointXYZINormal>::Ptr edge_pointsY(new pcl::PointCloud<pcl::PointXYZINormal>);

Livox_feature *livox_feature;
Cut_float *cutfloat;
Norm_est *norm_est;
GroundPlaneFit *groundfit;

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
    begin_time = clock();
    /*****creat variable**********/
    pcl::PointCloud<PointType> laserCloudIn;
    pcl::PointCloud<PointType> surfPointsFlat;
    pcl::PointCloud<PointType> cornerPointsSharp;
    pcl::VoxelGrid<PointType> sor;

    /*****clear variable value********/
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
    laserCloudFullRes_temp->clear();
    cloud_in_temp->clear();
    cloud_normals->clear();
    cloud_ground->clear();
    cloud_no_ground->clear();
    cloud_top->clear();
    intespoints_used_test->clear();
    edge_points->clear();
    edge_pointsY->clear();

   /***************process data*********/
    livox_feature->extract_feature(laserCloudIn,surfPointsFlat, cornerPointsSharp);

    pcl::copyPointCloud(laserCloudIn, *laserCloudFullRes_temp);
    std::vector<int> temp_ves;
    pcl::removeNaNFromPointCloud(*laserCloudFullRes_temp, *laserCloudFullRes_temp, temp_ves);
    //std::cout << "removeNaN 3974: " << cloud_in->points[3974] << endl;
    PointHandle::removeClosedPointCloud(*laserCloudFullRes_temp, *laserCloudFullRes_temp, 0.1);
    //cout << "size: " << laserCloudFullRes_temp->points.size() << endl;
   PointHandle::removeFarestPointCloud(*laserCloudFullRes_temp, *laserCloudFullRes_temp, 30);
  // cout << "size1: " << laserCloudFullRes_temp->points.size() << endl;

    sor.setInputCloud(laserCloudFullRes_temp);
    sor.setLeafSize(0.03f, 0.03f, 0.03f);
    sor.filter(*laserCloudFullRes_temp);

    //cout << "laserCloudFullRes size: " << laserCloudFullRes_temp->size() << endl;

    norm_est->Put_points(laserCloudFullRes_temp, cloud_normals);
    groundfit->put_pointsCloud(laserCloudFullRes_temp);

    pcl::copyPointCloud(*laserCloudFullRes_temp, *cloud_in_temp);

    Eigen::Vector3d reference_ground(0, 0, 1);
    groundfit->fitlerGroundWithNormal(cloud_in_temp, reference_ground, 0.4);
    groundfit->fitlerTopWithNormal(cloud_in_temp, -reference_ground, 0.4);

    //cout << "cloud normals size: " << cloud_normals->size() << endl;
    //cout << "cloud in temp size: " << cloud_in_temp->size() << endl;

    for (auto it: cloud_in_temp->points) {
        switch (int(it.curvature)) {
            case Cube_namespace::POINT_LABLE::GROUND :
                cloud_ground->points.push_back(it);
                break;

            case Cube_namespace::POINT_LABLE::NO_GROUND :
                cloud_no_ground->points.push_back(it);
                break;

            case Cube_namespace::POINT_LABLE::TOP :
                cloud_top->points.push_back(it);
                break;
        }
    }

    pcl::PointCloud<PointType> cornerPointsSharpGroudFiltered;

    livox_feature->remove_sharp_in_ground(cornerPointsSharp, *cloud_ground,
                                         cornerPointsSharpGroudFiltered, 0.15, 3);
    //pcl::copyPointCloud(cornerPointsSharp, cornerPointsSharpGroudFiltered);
   // cout << cornerPointsSharp.points.size() << " ------> " << cornerPointsSharpGroudFiltered.points.size() << endl;

    /********cut_float process******************/
    cutfloat->Put_points(cloud_in_temp);

    //cutfloat.get_bin_clouds(intespoints_used_test, Cut_namespace::DireX);
    cutfloat->RANSANCE_Estimate(10, Cut_namespace::DireX);
    cutfloat->Cal_Edge_points(0, Cut_namespace::DireX);
    cutfloat->Get_Edge_points(edge_points, Cut_namespace::DireX, 0.1);
    //cout << "edge_points: " << edge_points->size() << endl;
    //cutfloat.get_bin_clouds(intespoints_used_testY, Cut_namespace::DireY);
    cutfloat->RANSANCE_Estimate(10, Cut_namespace::DireY);
    cutfloat->Cal_Edge_points(0, Cut_namespace::DireY);
    cutfloat->Get_Edge_points(edge_pointsY, Cut_namespace::DireY, 0.1);
    //cout << "edge_pointsY: " << edge_pointsY->size() << endl;
/*
                    for (auto points: cloud_in_temp->points) {
                        points.curvature = 0;
                        intespoints_used_test->push_back(points);
                    }
                    */

    *intespoints_used_test += *edge_points;
    *intespoints_used_test += *edge_pointsY;

    cornerPointsSharpGroudFiltered += *intespoints_used_test;

    double mseconds = double(clock() - begin_time) / CLOCKS_PER_SEC;
    cout << "duration: " << mseconds << endl;
    /***********publish ros topic******/
  sensor_msgs::PointCloud2 laserCloudOutMsg;
  pcl::toROSMsg(*laserCloudFullRes_temp, laserCloudOutMsg);
  laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
  laserCloudOutMsg.header.frame_id = "zy_ka";
  pubLaserCloud.publish(laserCloudOutMsg);

  sensor_msgs::PointCloud2 cornerPointsSharpMsg;
  pcl::toROSMsg(cornerPointsSharpGroudFiltered, cornerPointsSharpMsg);
  cornerPointsSharpMsg.header.stamp = laserCloudMsg->header.stamp;
  cornerPointsSharpMsg.header.frame_id = "zy_ka";
  pubCornerPointsSharp.publish(cornerPointsSharpMsg);

  sensor_msgs::PointCloud2 surfPointsFlat2;
  pcl::toROSMsg(surfPointsFlat, surfPointsFlat2);
  surfPointsFlat2.header.stamp = laserCloudMsg->header.stamp;
  surfPointsFlat2.header.frame_id = "zy_ka";
  pubSurfPointsFlat.publish(surfPointsFlat2);

  sensor_msgs::PointCloud2 edgePoints;
  pcl::toROSMsg(*intespoints_used_test, edgePoints);
  edgePoints.header.stamp = laserCloudMsg->header.stamp;
  edgePoints.header.frame_id = "zy_ka";
  pubEdgPoints.publish(edgePoints);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "zy_scanRegistration");
  ros::NodeHandle nh;

  // ros::Subscriber subLaserCloud_for_hk = nh.subscribe<sensor_msgs::PointCloud2>
  //                                 ("/livox/lidar", 2, laserCloudHandler_temp);
  // pubLaserCloud_for_hk = nh.advertise<sensor_msgs::PointCloud2>
  //                                ("/livox/lidar_temp", 2);

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>
                                  ("/zy/livox_pcl0", 10000, laserCloudHandler);

  pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>
                                 ("/zy/livox_cloud", 100);
  pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>
                                        ("/zy/laser_cloud_sharp", 100);
  pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>
                                       ("/zy/laser_cloud_flat", 100);
  pubEdgPoints =  nh.advertise<sensor_msgs::PointCloud2>
            ("/zy/laser_cloud_edge", 100);

  livox_feature = new Livox_feature();
  //cutfloat = new Cut_float(0.15, 0.05, 130, 100);
  cutfloat = new Cut_float(0.10, 0.05, 130, 100);
  norm_est = new Norm_est(20);
  groundfit = new GroundPlaneFit();

  ros::spin();

  return 0;
}
