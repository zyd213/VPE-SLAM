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

#define  PI  3.1415926535
#define Write_frame true
#define Frame_PATH "/home/adminpc/zy_ka/src/zy_ka/data/test_data.pcd"

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

    std::vector<int> temp_ves;

    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, temp_ves);
    //std::cout << "removeNaN 3974: " << cloud_in->points[3974] << endl;
    PointHandle::removeClosedPointCloud(*cloud_in, *cloud_in, 0.1);

    Norm_est norm_est(15);
    GroundPlaneFit groundfit = GroundPlaneFit();
    groundfit.put_pointsCloud(cloud_in);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_in_temp(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_no_ground(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_top(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_cube_out(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_cube_out1(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr intespoints_used_test(new pcl::PointCloud<pcl::PointXYZINormal>);

    pcl::copyPointCloud(*cloud_in, *cloud_in_temp);
    norm_est.Put_points(cloud_in_temp, cloud_normals);

    Eigen::Vector3d reference_ground(0, 0, 1);
    groundfit.fitlerGroundWithNormal(cloud_in_temp, reference_ground, 0.5);
    groundfit.fitlerTopWithNormal(cloud_in_temp, -reference_ground, 0.5);

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


    Cube_float_super super_cube(2, 150, 40);
    cout << "creat: " << endl;
    super_cube.Put_points(cloud_in_temp);
    cout << "put_points: " << endl;
    super_cube.plan_intersection_points_cal(cloud_no_ground, cloud_ground,
                                         1, 0.1);
    super_cube.plan_intersection_points_cal(cloud_no_ground, cloud_top,
                                         1, 0.1);
    cout << "intersection: " << endl;
    super_cube.cube_fit(0.2, 3);
    cout << "end" << endl;

    cout << "(((((((((((((((((((" << endl;
    cout << "get_pointsNum of occupyed bin: " << super_cube.get_pointsNum_of_occupyed_bin() << endl;
    cout << "$$$$$$$$$$$$$$$$$$" << endl;
    super_cube.get_intesPoints_cross_bin_vector_by_different_caverature(intespoints_used_test);

    cout << "********************" << endl;
    cout << "get intesPoints by viewer: size: " << intespoints_used_test->points.size() << endl;
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZINormal> single_color(intespoints_used_test, "curvature");
    boost::shared_ptr<pcl::visualization::PCLVisualizer>  viewer1, viewer2, viewer3;

    viewer1 = customColourVis(intespoints_used_test);
    viewer1->updatePointCloud<pcl::PointXYZINormal>(intespoints_used_test, single_color, "zy_cloud");
    super_cube.viewer_add_cube(viewer1);
    //cube_float.viewer_add_points_in_cent(viewer1);
    /*
    cube_float.viewer_add_cube(viewer1);

    cube_float.clear_occupyed_bin_low_num(4);
    cout << "num_bin: " << cube_float.get_bin_num_occupyed() << endl;

    cube_float.get_points_of_occupyed_bin(cloud_cube_out1);

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZINormal> single_color1(cloud_cube_out1, "curvature");
    viewer2 = customColourVis(cloud_cube_out1);
    viewer2->updatePointCloud<pcl::PointXYZINormal>(cloud_cube_out1, single_color1, "zy_cloud");
    //cube_float.viewer_add_points_in_cent(viewer1);
    cube_float.viewer_add_cube(viewer2);
*/
   //viewer1 =  customColourVisCompare(cloud_in, cloud_in_temp, "port1", "port2");
    while(true)
    {
        if (!viewer1->wasStopped())
        {
            viewer1->spinOnce(1);
        }
        /*
        if (!viewer2->wasStopped())
        {
           // viewer2->spinOnce(1);
        }
         */

    }
    return 0;
}

/*
 * cube_float 分析 2021年11月23日
 *     pcl::PointCloud<PointType>::Ptr cloud_in(new pcl::PointCloud<PointType>);
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

    std::vector<int> temp_ves;

    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, temp_ves);
    //std::cout << "removeNaN 3974: " << cloud_in->points[3974] << endl;
    PointHandle::removeClosedPointCloud(*cloud_in, *cloud_in, 0.1);

    Norm_est norm_est(15);
    GroundPlaneFit groundfit = GroundPlaneFit();
    groundfit.put_pointsCloud(cloud_in);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_in_temp(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_no_ground(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_top(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_cube_out(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_cube_out1(new pcl::PointCloud<pcl::PointXYZINormal>);

    pcl::copyPointCloud(*cloud_in, *cloud_in_temp);
    norm_est.Put_points(cloud_in_temp, cloud_normals);

    Eigen::Vector3d reference_ground(0, 0, 1);
    groundfit.fitlerGroundWithNormal(cloud_in_temp, reference_ground, 0.5);
    groundfit.fitlerTopWithNormal(cloud_in_temp, -reference_ground, 0.5);

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

    Cube_float cube_float(2, 150);

    cube_float.Put_points(cloud_in_temp);
    cube_float.plan_intersection_points_cal(cloud_no_ground, cloud_ground,
                                         1, 0.1);
    cube_float.plan_intersection_points_cal(cloud_no_ground, cloud_top,
                                         1, 0.1);
    //cout << "num_bin: " << cube_float.get_bin_num_occupyed() << endl;
    //cube_float.put_points_in_occupyed_cube();

    cout << "num_bin: " << cube_float.get_bin_num_occupyed() << endl;
    cout << "num_points: " << cube_float.get_pointsNum_of_occupyed_bin() << endl;
    cube_float.get_points_of_occupyed_bin(cloud_cube_out);

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZINormal> single_color(cloud_cube_out, "curvature");
    boost::shared_ptr<pcl::visualization::PCLVisualizer>  viewer1, viewer2, viewer3;
    viewer1 = customColourVis(cloud_cube_out);
    viewer1->updatePointCloud<pcl::PointXYZINormal>(cloud_cube_out, single_color, "zy_cloud");
    //cube_float.viewer_add_points_in_cent(viewer1);
    cube_float.viewer_add_cube(viewer1);

    cube_float.clear_occupyed_bin_low_num(4);
    cout << "num_bin: " << cube_float.get_bin_num_occupyed() << endl;

    cube_float.get_points_of_occupyed_bin(cloud_cube_out1);

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZINormal> single_color1(cloud_cube_out1, "curvature");
    viewer2 = customColourVis(cloud_cube_out1);
    viewer2->updatePointCloud<pcl::PointXYZINormal>(cloud_cube_out1, single_color1, "zy_cloud");
    //cube_float.viewer_add_points_in_cent(viewer1);
    cube_float.viewer_add_cube(viewer2);



    while(true)
    {
        if (!viewer1->wasStopped())
        {
            viewer1->spinOnce(1);
        }
        if (!viewer2->wasStopped())
        {
           // viewer2->spinOnce(1);
        }

    }
    return 0;
}
 */



/*
 * cube_float 分析 2021年11月17日
 *
 *   pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZINormal> single_color(cloud_cube_out, "curvature");
    boost::shared_ptr<pcl::visualization::PCLVisualizer>  viewer1, viewer2, viewer3;
    viewer1 = customColourVis(cloud_cube_out);
    viewer1->updatePointCloud<pcl::PointXYZINormal>(cloud_cube_out, single_color, "zy_cloud");
    //cube_float.viewer_add_points_in_cent(viewer1);
    cube_float.viewer_add_cube(viewer1);

    cube_float.clear_occupyed_bin_low_num(4);
    cout << "num_bin: " << cube_float.get_bin_num_occupyed() << endl;

    cube_float.get_points_of_occupyed_bin(cloud_cube_out1);

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZINormal> single_color1(cloud_cube_out1, "curvature");
    viewer2 = customColourVis(cloud_cube_out1);
    viewer2->updatePointCloud<pcl::PointXYZINormal>(cloud_cube_out1, single_color1, "zy_cloud");
    //cube_float.viewer_add_points_in_cent(viewer1);
    cube_float.viewer_add_cube(viewer2);
 */
/*****************20211104**********************/
/*
 * cube 分析
 */
/*
int main(int argc, char** argv) {
    pcl::PointCloud<PointType>::Ptr cloud_in(new pcl::PointCloud<PointType>);
    if (pcl::io::loadPCDFile<PointType>(Frame_PATH, *cloud_in) == -1)
    {
        PCL_ERROR("Couldn't read file test_data \n");
        return (-1);
    }
    std::cout << "Loaded "
              << cloud_in->width * cloud_in->height
              << " data points from test_pcd.pcd with the following fields: "
              << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr mls_cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr mls_cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < cloud_in->points.size(); i++)
    {
        pcl::PointXYZ temp_point;
        temp_point.x = cloud_in->points[i].x;
        temp_point.y = cloud_in->points[i].y;
        temp_point.z = cloud_in->points[i].z;
        mls_cloud_in->points.push_back(temp_point);
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_UP(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    mls.setInputCloud(mls_cloud_in);
    mls.setSearchMethod(tree_UP);
    mls.setSearchRadius(1);
    mls.setPolynomialFit(true);
    mls.setComputeNormals(true);
    mls.process(*mls_cloud_out);

    pcl::PointCloud<PointType>::Ptr cloud_UP_type(new pcl::PointCloud<PointType>());
    for (int i = 0; i < mls_cloud_out->points.size(); i++)
    {
        PointType temp_point;
        temp_point.x = mls_cloud_out->points[i].x;
        temp_point.y = mls_cloud_out->points[i].y;
        temp_point.z = mls_cloud_out->points[i].z;
        cloud_UP_type->points.push_back(temp_point);
    }

    Cube_cla cube_cla(1, 20);
    std::cout << cube_cla.cube_vec.size() << std::endl;
    cube_cla.Put_points(cloud_in);
    std::cout << "put_points " << endl;
    std::vector<int> id_vec = cube_cla.get_id_vec();
    std::cout << "id_vec size: " << id_vec.size() << endl;

    pcl::PointCloud<PointType>::Ptr viewer2_points(new pcl::PointCloud<PointType>);

    double intensity_step = 30;
    int id_vec_cnt  = 0;

    for (auto it: id_vec)
    {
        id_vec_cnt++;
        if (id_vec_cnt%2 == 0)
        {
            for (int i = 0; i < cube_cla.cube_vec[it]->points.size(); i++)
            {
                PointType temp_point;
                temp_point.x = cube_cla.cube_vec[it]->points[i].x;
                temp_point.y = cube_cla.cube_vec[it]->points[i].y;
                temp_point.z = cube_cla.cube_vec[it]->points[i].z;
                temp_point.intensity =  id_vec_cnt * intensity_step;

                viewer2_points->points.push_back(temp_point);
            }
        }

       //*viewer2_points += *cube_cla.cube_vec[it];
    }
    std::cout << "view 1 Points size: " << cloud_in->points.size() << endl;
    std::cout << "view 2 Points size: " << viewer2_points->points.size() << endl;
   // std::cout << "view 3 Points size: " << cloud_UP_type->points.size() << endl;
    boost::shared_ptr<pcl::visualization::PCLVisualizer>  viewer1, viewer2, viewer3;
    viewer1 = customColourVis(cloud_in);
    //viewer2 = customColourVis(cube_cla.cube_vec[id_vec[20]]);
    viewer2 = customColourVis(viewer2_points);
   // viewer3 = customColourVis(cloud_UP_type);

    while(true)
    {
        if (!viewer1->wasStopped())
        {
            viewer1->spinOnce(100);
        }
        if (!viewer2->wasStopped())
        {
            viewer2->spinOnce(100);
        }
        /*
        if (!viewer3->wasStopped())
        {
            viewer3->spinOnce(100);
        }
         */
        //boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        //boost::this_thread::sleep(boost::posix_time::microseconds(1000000));
        /*
    }
return 0;
}
*/
/*****************20211104***************************/