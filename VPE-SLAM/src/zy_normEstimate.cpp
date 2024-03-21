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
#include <std_msgs/Header.h>
#include <eigen3/Eigen/Core>
#include "utils/Entity_Cla.h"
#include "utils/FastGroundFit.h"
#include "utils/pcl_viewer.h"

const bool viewer_enable = true;
int N_SCANS = 6;
float timeLaserCloudCorner = 0;
float timeLaserCloudSurf = 0;
float timeLaserCloudFullRes = 0;

bool newLaserCloudCorner = false;
bool newLaserCloudSurf = false;
bool newLaserCloudFullRes = false;
bool first_frame = false;

std_msgs::Header header;
//corner feature
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

void laserCloudCornerHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudCornerLast2)
{
    timeLaserCloudCorner = laserCloudCornerLast2->header.stamp.toSec();

    laserCloudCorner->clear();
    pcl::fromROSMsg(*laserCloudCornerLast2, *laserCloudCorner);

    newLaserCloudCorner= true;


}

void laserCloudSurfHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudSurfLast2)
{
    timeLaserCloudSurf= laserCloudSurfLast2->header.stamp.toSec();

    laserCloudSurf->clear();
    pcl::fromROSMsg(*laserCloudSurfLast2, *laserCloudSurf);

    newLaserCloudSurf= true;
}

void laserCloudFullHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullRes2)
{
    timeLaserCloudFullRes = laserCloudFullRes2->header.stamp.toSec();
    header = laserCloudFullRes2->header;
    laserCloudFullRes->clear();
    pcl::fromROSMsg(*laserCloudFullRes2, *laserCloudFullRes);
    ROS_INFO("laserCloudFullReceived!");
    newLaserCloudFullRes = true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "zy_normEstimate");
    ros::NodeHandle nh;

    ros::Subscriber subLaserCloudCor = nh.subscribe<sensor_msgs::PointCloud2>
            ("zy/laser_cloud_sharp", 100, laserCloudCornerHandler);
    ros::Subscriber subLaserCloudFlat = nh.subscribe<sensor_msgs::PointCloud2>
            ("zy/laser_cloud_flat", 100, laserCloudSurfHandler);
    ros::Subscriber subLaserCloudFull = nh.subscribe<sensor_msgs::PointCloud2>
            ("zy/livox_cloud", 100, laserCloudFullHandler);

    ros::Publisher pubGroundPoint = nh.advertise<sensor_msgs::PointCloud2>("zy/nEGroundPoint", 100);
    ros::Publisher pubFullCloud = nh.advertise<sensor_msgs::PointCloud2>("zy/normFullPoints", 100);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    ros::Rate rate(100);

    Cut_float cutfloat(0.15, 0.05, 130, 100);
    Norm_est norm_est(20);
    GroundPlaneFit groundfit = GroundPlaneFit();
    ROS_INFO("\033[1;32m---->\033[0m zyd Norm seg Started!");
    pcl::VoxelGrid<pcl::PointXYZINormal> sor;
    int sys_cnt = 0;
    const int one = 10;
    const int two = 11;

    while (ros::ok()) {
        ros::spinOnce();
        //ROS_INFO("spinOnce");
        if (newLaserCloudFullRes) {
            newLaserCloudFullRes = false;
            cout << "cnt : " << sys_cnt++ << endl;
            if (first_frame == false)
            {
                viewer = customColourVis(laserCloudFullRes);
            }
            first_frame = true;
            //if (sys_cnt == one) {
               // if (sys_cnt == one || sys_cnt == two) {
                    laserCloudFullRes_temp->clear();
                    cloud_in_temp->clear();
                    cloud_normals->clear();
                    cloud_ground->clear();
                    cloud_no_ground->clear();
                    cloud_top->clear();
                    intespoints_used_test->clear();
                    edge_points->clear();
                    edge_pointsY->clear();
                    std::cout << "************" << endl;


                    pcl::copyPointCloud(*laserCloudFullRes,
                                        *laserCloudFullRes_temp); /*  *****fig ground test **************/
                    std::vector<int> temp_ves;
                    pcl::removeNaNFromPointCloud(*laserCloudFullRes_temp, *laserCloudFullRes_temp, temp_ves);
                    //std::cout << "removeNaN 3974: " << cloud_in->points[3974] << endl;
                    PointHandle::removeClosedPointCloud(*laserCloudFullRes_temp, *laserCloudFullRes_temp, 0.1);
                    sor.setInputCloud(laserCloudFullRes_temp);
                    sor.setLeafSize(0.03f, 0.03f, 0.03f);
                    sor.filter(*laserCloudFullRes_temp);

                    cout << "laserCloudFullRes size: " << laserCloudFullRes_temp->size() << endl;

                    norm_est.Put_points(laserCloudFullRes_temp, cloud_normals);
/*
                    if (sys_cnt == two) {
                        // viewer = customColourVis(laserCloudFullRes);
                        viewer = normalsVis(laserCloudFullRes_temp, cloud_normals);
                        //groundFit.viewer = viewer;
                    }
                    */

                    groundfit.put_pointsCloud(laserCloudFullRes_temp);
                    pcl::copyPointCloud(*laserCloudFullRes_temp, *cloud_in_temp);
/*
                pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZINormal> single_color(cloud_in_temp,
                                                                                                          "curvature");//点云颜色设置
                viewer->updatePointCloud<pcl::PointXYZINormal>(cloud_in_temp, single_color, "zy_cloud");
                */


                    Eigen::Vector3d reference_ground(0, 0, 1);

                    groundfit.fitlerGroundWithNormal(cloud_in_temp, reference_ground, 0.4);
                    groundfit.fitlerTopWithNormal(cloud_in_temp, -reference_ground, 0.4);

                    cout << "cloud normals size: " << cloud_normals->size() << endl;
                    cout << "cloud in temp size: " << cloud_in_temp->size() << endl;

/*
                    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZINormal> single_color(cloud_in_temp,
                                                                                                              "curvature");//点云颜色设置
                    viewer->updatePointCloud<pcl::PointXYZINormal>(cloud_in_temp, single_color, "zy_cloud");
                    */

                    //std::cout << "Cube_float1: " << endl;
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


                    cout << "gournd size: " << cloud_ground->points.size() << endl;
                    cout << "no ground size: " << cloud_no_ground->size() << endl;
                    cout << "top size: " << cloud_top->size() << endl;

                    cout << "cloud in temp size: " << cloud_in_temp->points.size() << endl;
                    cutfloat.Put_points(cloud_in_temp);

                    //cutfloat.get_bin_clouds(intespoints_used_test, Cut_namespace::DireX);
                    cutfloat.RANSANCE_Estimate(10, Cut_namespace::DireX);
                    cutfloat.Cal_Edge_points(0, Cut_namespace::DireX);
                    cutfloat.Get_Edge_points(edge_points, Cut_namespace::DireX, 0.1);
                    cout << "edge_points: " << edge_points->size() << endl;
                    //cutfloat.get_bin_clouds(intespoints_used_testY, Cut_namespace::DireY);
                    cutfloat.RANSANCE_Estimate(10, Cut_namespace::DireY);
                    cutfloat.Cal_Edge_points(0, Cut_namespace::DireY);
                    cutfloat.Get_Edge_points(edge_pointsY, Cut_namespace::DireY, 0.1);
                    cout << "edge_pointsY: " << edge_pointsY->size() << endl;
/*
                    for (auto points: cloud_in_temp->points) {
                        points.curvature = 0;
                        intespoints_used_test->push_back(points);
                    }
                    */

                    *intespoints_used_test += *edge_points;
                    *intespoints_used_test += *edge_pointsY;

                    sensor_msgs::PointCloud2 pcl_ros_msg;
                    pcl::toROSMsg(*intespoints_used_test, pcl_ros_msg);
                    pcl_ros_msg.header = header;
                    pubGroundPoint.publish(pcl_ros_msg);

                    pcl::toROSMsg(*laserCloudFullRes_temp, pcl_ros_msg);
                    pcl_ros_msg.header = header;
                    pubFullCloud.publish(pcl_ros_msg);

/*
                pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZINormal> single_color(intespoints_used_test,
                                                                                                          "curvature");//点云颜色设置
                viewer->updatePointCloud<pcl::PointXYZINormal>(intespoints_used_test, single_color, "zy_cloud");
                */



                    /*
                    sensor_msgs::PointCloud2 pointCloud_Sensor;
                    pcl::toROSMsg(*laserCloudGround, pointCloud_Sensor);
                    pointCloud_Sensor.header = header;
                    pubGroundPoint.publish(pointCloud_Sensor);
                    */
                //}
/*
                if (!viewer->wasStopped()) {
                    viewer->spinOnce(1);
                } else {
                    break;
                }
                */
            }

        //rate.sleep();
        //ROS_INFO("Sleep");
    }
    return 0;
}

/*
while (ros::ok())
{
ros::spinOnce();
// ROS_INFO("spinOnce");
if (newLaserCloudCorner && newLaserCloudSurf && newLaserCloudFullRes &&
        fabs(timeLaserCloudSurf - timeLaserCloudCorner) < 0.005 &&
fabs(timeLaserCloudFullRes - timeLaserCloudCorner) < 0.005) {

if (!first_frame)
{
viewer = customColourVis(laserCloudGround);
groundFit.viewer = viewer;
}
first_frame = true;

if (laserCloudFullRes->points.size() > 10)
{
std::vector<int> ground_index;
//std::cout << "laserCloudFull size: " << laserCloudFullRes->points.size() << endl;

ground_index = groundFit.put_pointsCloud(laserCloudFullRes);
//std::cout << "ground_index size: " << ground_index.size() << endl;

laserCloudGround->clear();
PointType temp_point;
pcl::copyPointCloud(*laserCloudFullRes, *laserCloudGround);
for (std::vector<int>::iterator it = ground_index.begin(); it < ground_index.end(); ++it)
{
laserCloudGround->points[*it].intensity = 200;
}
pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZINormal> single_color(laserCloudGround, "intensity");//点云颜色设置
viewer->updatePointCloud<pcl::PointXYZINormal>(laserCloudGround, single_color,"zy_cloud");

if (!viewer->wasStopped())
{
viewer->spinOnce(1);
} else
{
break;
}

sensor_msgs::PointCloud2 pointCloud_Sensor;
pcl::toROSMsg(*laserCloudGround, pointCloud_Sensor);
pointCloud_Sensor.header = header;
pubGroundPoint.publish(pointCloud_Sensor);

} else{
std::cout << "PointClouds size less than ten" << endl;
}

newLaserCloudCorner = false;
newLaserCloudSurf = false;
newLaserCloudFullRes = false;
}
*/