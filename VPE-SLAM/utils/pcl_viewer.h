#ifndef _PCL_VIEWER_H
#define _PCL_VIEWER_H

#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
/*****pcl viewer************/
/*
template <typename PointT>
boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVisCompare(
        const typename pcl::PointCloud<PointT>::ConstPtr &cloud1,
        const typename pcl::PointCloud<PointT>::ConstPtr &cloud2,
        const std::string &id_view1,
        const std::string &id_view2);
        */

boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVisCompare(
         pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr cloud1,
         pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr cloud2,
        const std::string &id_view1,
        const std::string &id_view2);
boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVisCompare(
        pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr cloud1,
        pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr cloud2,
        const std::string &id_view1,
        const std::string &id_view2,
        const std::string &title1,
        const std::string &title2);

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr cloud);
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis(pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr cloud);
boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis(
        pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals);
boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis(
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
        pcl::PointCloud<pcl::Normal>::ConstPtr normals1,
        pcl::PointCloud<pcl::Normal>::ConstPtr normals2);
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
                           void *viewer_void);
void mouseEventOccurred(const pcl::visualization::MouseEvent &event,
                        void *viewer_void);
boost::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis();
/******************/
#endif