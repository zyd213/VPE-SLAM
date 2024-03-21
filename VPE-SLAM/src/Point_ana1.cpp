#include "livox_ros_driver/CustomMsg.h"
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/surface/mls.h>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include "utils/pcl_viewer.h"
#include "iostream"
#include <stdio.h>      /* printf */
#include <math.h>       /* fmod */
#include "utils/Entity_Cla.h"
#include "utils/FastGroundFit.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud

#define  PI  3.1415926535
#define Write_frame true
#define Frame_PATH "/home/adminpc/zy_ka/src/zy_ka/data/test_data30.pcd"

int main(int argc, char** argv) {
    pcl::PointCloud<PointType>::Ptr cloud_in(new pcl::PointCloud<PointType>);
    if (pcl::io::loadPCDFile<PointType>(Frame_PATH, *cloud_in) == -1) {
        PCL_ERROR("Couldn't read file test_data \n");
        return (-1);
    }
    std::cout << "Loaded "
              << cloud_in->width * cloud_in->height
              << " data points from test_pcd.pcd with the following fields: "
              << std::endl;

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_fitlered(new pcl::PointCloud<pcl::PointXYZINormal>);

    pcl::VoxelGrid<pcl::PointXYZINormal> sor;

    std::vector<int> temp_ves;

    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, temp_ves);
    //std::cout << "removeNaN 3974: " << cloud_in->points[3974] << endl;
    PointHandle::removeClosedPointCloud(*cloud_in, *cloud_in, 0.1);
    sor.setInputCloud(cloud_in);
    sor.setLeafSize(0.05f, 0.05f, 0.05f);
    sor.filter(*cloud_in);
    cout << "cloud in size: " << cloud_in->points.size() << endl;
    Norm_est norm_est(15);

    norm_est.Put_points(cloud_in, cloud_normals);
    GroundPlaneFit groundfit = GroundPlaneFit();
    groundfit.put_pointsCloud(cloud_in);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_in_temp(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_no_ground(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_top(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_cube_out(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_cube_out1(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr intespoints_used_test(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr intespoints_used_testY(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr test_used_as_seg(new pcl::PointCloud<pcl::PointXYZINormal>);

    pcl::copyPointCloud(*cloud_in, *cloud_in_temp);

    Eigen::Vector3d reference_ground(0, 0, 1);
    groundfit.fitlerGroundWithNormal(cloud_in_temp, reference_ground, 0.5);
    groundfit.fitlerTopWithNormal(cloud_in_temp, -reference_ground, 0.5);

    cout << "cloud normals size: " << cloud_normals->size() << endl;
    cout << "cloud in temp size: " << cloud_in_temp->size() << endl;

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
    Cut_float cutfloat(0.08, 0.04, 250, 200);
    cutfloat.Put_points(cloud_in_temp);

    int test_id = 16;
    //cutfloat.get_bin_clouds(intespoints_used_test, Cut_namespace::DireX, test_id);
   // cutfloat.RANSANCE_Estimate(test_id, Cut_namespace::DireX, 5, 4);

    cutfloat.get_bin_clouds(intespoints_used_test, Cut_namespace::DireX);
    cutfloat.RANSANCE_Estimate(10,  Cut_namespace::DireX);
    cutfloat.Cal_Edge_points(0, Cut_namespace::DireX);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr edge_points(new pcl::PointCloud<pcl::PointXYZINormal>);
    cutfloat.Get_Edge_points(edge_points, Cut_namespace::DireX, 0.1);
    *intespoints_used_test += *edge_points;
    cout << "get bin clouds size: " << intespoints_used_test->size() << endl;

    cutfloat.get_bin_clouds(intespoints_used_testY, Cut_namespace::DireY);
    cutfloat.RANSANCE_Estimate(10,  Cut_namespace::DireY);
    cutfloat.Cal_Edge_points(0, Cut_namespace::DireY);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr edge_pointsY(new pcl::PointCloud<pcl::PointXYZINormal>);
    cutfloat.Get_Edge_points(edge_pointsY, Cut_namespace::DireY, 0.1);
    cout << "edge_pointsY: " << edge_pointsY->size() << endl;
    *intespoints_used_testY += *edge_pointsY;
    cout << "get bin clouds size Y: " << intespoints_used_testY->size() << endl;

    intespoints_used_test->clear();
    for (auto points : cloud_in_temp->points)
    {
        points.curvature = 10;
        intespoints_used_test->push_back(points);
    }

    *intespoints_used_test += *edge_points;
    *intespoints_used_test += *edge_pointsY;

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZINormal> single_color(intespoints_used_test, "curvature");
    boost::shared_ptr<pcl::visualization::PCLVisualizer>  viewer1, viewer2, viewer3;

    viewer1 = customColourVis(intespoints_used_test);
    viewer1->updatePointCloud<pcl::PointXYZINormal>(intespoints_used_test, single_color, "zy_cloud");
    viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "zy_cloud");

    //cutfloat.Draw_line(viewer1, Cut_namespace::DireY);

//viewer1 =  customColourVisCompare(cloud_in, cloud_in_temp, "port1", "port2");
    while(true) {
        if (!viewer1->wasStopped())
        {
            viewer1->spinOnce(1);
        }

/*
        if (!viewer.wasStopped())
        {
          // viewer.spinOnce(1);
        }
        */


    }
    return 0;
}


/*****************20211104***************************/
