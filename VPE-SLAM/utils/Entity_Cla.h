#ifndef Entity_Cla_H
#define Entity_Cla_H

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/surface/mls.h>
#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include "utils/pcl_viewer.h"
#include <stdio.h>      /* printf */
#include <math.h>       /* fmod */
#include "utils/common.h"
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/region_growing.h>
#include <opencv/cv.h>

class Cube_float;

namespace PointHandle
{
    void removeClosedPointCloud(const pcl::PointCloud<pcl::PointXYZINormal> &cloud_in, pcl::PointCloud<pcl::PointXYZINormal> &cloud_out, float thres);
    void removeFarestPointCloud(const pcl::PointCloud<pcl::PointXYZINormal> &cloud_in, pcl::PointCloud<pcl::PointXYZINormal> &cloud_out, float thres);
}

namespace Cube_namespace
{
    enum   POINT_LABLE
    {
        GROUND = 0,
        NO_GROUND = 15,
        TOP = 30,
        EDGE = 45
    };

    struct id_stc{
        int id;
        int num_all;
        int num_ground;
        int num_no_ground;
        int num_top;
        double radio;
    };

    struct COMPONT_RADIO{
        int id;
        double radio;
    };

    struct cube_float_bin{
      int id;
      pcl::PointCloud<pcl::PointXYZINormal>::Ptr points;
      double Cent_x;
      double Cent_y;
      double Cent_z;
      double begin_x, end_x;
      double begin_y, end_y;
      double begin_z, end_z;
      bool used;
      Cube_namespace::id_stc compont;
    };

    struct cross_bin
    {
       Cube_float * cube_float;
       bool used = false;
    };
}

namespace Cut_namespace
{
    enum  DIRECTORION
    {
        DireX = 0,
        DireY
    };
}
namespace Livox_namespace
{
    //extern int CSANC;

    enum POINT_LABLE{
        flat = 1,
        sharp0 = 100,
        sharp1 = 150,
         out_lier = 250,
    };
}

class Cut_bin
{
public:
      pcl::PointCloud<pcl::PointXYZINormal>::Ptr bin_points;
      pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_top;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_ground;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_no_ground;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr RANS_EDGE_cloud;

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_temp;

    vector<Eigen::VectorXf> ground_cofficient_vector;
    vector<Eigen::VectorXf> no_ground_cofficient_vector;
    vector<Eigen::VectorXf> top_cofficient_vector;

    pcl::SampleConsensusModel<pcl::PointXYZINormal>::Ptr conse_model;
    pcl::RandomSampleConsensus<pcl::PointXYZINormal>::Ptr conse;

    Cut_namespace::DIRECTORION dire;
      double Cent;
      double Cent_range;
      Cut_bin(double Cent, double Cent_range, Cut_namespace::DIRECTORION dire);
    void Put_points(pcl::PointCloud<pcl::PointXYZINormal>::Ptr point_in);
    void RANSANCE_Estimate(int min_points_num, int max_iterate);
    void RANSANCE_Estimate(int min_points_num );

    void Cal_Edge_points(double th_edge );

private:
    double inline_points_th = 5;
    double outliers_points_th = 10;
    double RAN_Cent_range_fit_radio = 2;
    int reg_min_size = 15;
    int regSearchKnumber = 30;
    double regSmoothnessTh = 8.0/180.0 * M_PI;
    double regCurvatureTh = 1;
    pcl::search::Search<pcl::PointXYZINormal>::Ptr tree;
    pcl::RegionGrowing<pcl::PointXYZINormal, pcl::Normal> reg;

    void RANSANCE_Estimate_specific(pcl::PointCloud<pcl::PointXYZINormal>::Ptr in_cloud,
                                    vector<Eigen::VectorXf> &cofficient,
                                    int &residual_points_size);
    void RANSANCE_Estimate_cluster(pcl::PointCloud<pcl::PointXYZINormal>::Ptr in_cloud,
                                   vector<Eigen::VectorXf> &cofficient);
    void clear_points(void);

    void Cal_edge_for_dirX(vector<Eigen::VectorXf> &coffe1,
                           vector<Eigen::VectorXf> &coffe2, double th);
    void Cal_edge_for_dirY(vector<Eigen::VectorXf> &coffe1,
                           vector<Eigen::VectorXf> &coffe2, double th);

};

class Cut_float
        {
        public:

            double Segment_interval;
            double Cent_range;
            double Cut_bin_dirX_num;
            double Cut_bin_dirY_num;

            std::vector<Cut_bin*> Cut_bin_dirX_vector;
           std::vector<Cut_bin*> Cut_bin_dirY_vector;

       Cut_float(double Segment_interval, double Cent_range, double Cut_bin_dirX_num, double Cut_bin_dirY_num);
        void Put_points(pcl::PointCloud<pcl::PointXYZINormal>::Ptr point_in);
        void get_bin_clouds(pcl::PointCloud<pcl::PointXYZINormal>::Ptr point_out, Cut_namespace::DIRECTORION dire);
        void get_bin_clouds(pcl::PointCloud<pcl::PointXYZINormal>::Ptr point_out, Cut_namespace::DIRECTORION dire, int specifi_id);
        void get_bin_clouds_specific(pcl::PointCloud<pcl::PointXYZINormal>::Ptr point_out, Cut_namespace::DIRECTORION dire);


    void RANSANCE_Estimate(int id, Cut_namespace::DIRECTORION dire,
                                      int min_points_num, int max_iterate);
    void RANSANCE_Estimate(int min_points_num, int max_iterate, Cut_namespace::DIRECTORION dire);
    void RANSANCE_Estimate(int min_points_num,  Cut_namespace::DIRECTORION dire);
    void Cal_Edge_points(double th_edge, Cut_namespace::DIRECTORION dire);
    void Get_Edge_points(pcl::PointCloud<pcl::PointXYZINormal>::Ptr point_out, Cut_namespace::DIRECTORION dire);
    void Get_Edge_points(pcl::PointCloud<pcl::PointXYZINormal>::Ptr point_out, Cut_namespace::DIRECTORION dire, double kn);

    void Draw_line(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
                   int id, Cut_namespace::DIRECTORION dire);
    void Draw_line(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
                    Cut_namespace::DIRECTORION dire);
private:
            int K_point_limit = 20;
            pcl::search::KdTree<pcl::PointXYZINormal>::Ptr edge_tree;
            double Centrange_Inc(double Cent_range, int id, int segment_num);

};

class Cube_float
{
public:
    std::vector<Cube_namespace::cube_float_bin> cube_float_bin_vec;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr intesertion_plan_point;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr intesertion_edge_point;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr points_in;

    Cube_float(double Sx, double Sy, double Sz, int bin_num);
    Cube_float(double leng_th, int bin_num);
    void Put_points(pcl::PointCloud<pcl::PointXYZINormal>::Ptr points_in);
    void put_intersection_points(pcl::PointCloud<pcl::PointXYZINormal>::Ptr in_points);

    void plan_intersection_points_cal(pcl::PointCloud<pcl::PointXYZINormal>::Ptr no_ground_points,
                               pcl::PointCloud<pcl::PointXYZINormal>::Ptr target,
                               int k_search, double radio_th);
    void plan_intersection_points_cal(pcl::PointCloud<pcl::PointXYZINormal>::Ptr no_ground_points,
                                               pcl::PointCloud<pcl::PointXYZINormal>::Ptr target,
                                               double radio_th);
    void plan_intersection_points_cal_bysuper(pcl::PointCloud<pcl::PointXYZINormal>::Ptr no_ground_points,
                                                       pcl::PointCloud<pcl::PointXYZINormal>::Ptr target,
                                                       int k_search, double radio_th);
    int get_bin_num_occupyed(void);
    int get_pointsNum_of_occupyed_bin(void);
    void get_points_of_occupyed_bin(pcl::PointCloud<pcl::PointXYZINormal>::Ptr bin_cloud);
    void clear_occupyed_bin_low_radio(double th);
    void clear_occupyed_bin_low_num(int points_num);

    void viewer_add_circle(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
    void viewer_add_cube(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);

    void put_points_in_occupyed_cube(void);
    void put_intersection_points_in_occupyed_cube(void);
    Cube_float operator=(Cube_float &cube_in);
    Cube_float (Cube_float& cube_in);

    double Sx, Sy, Sz;
    int bin_num;
    double next_empty_id;
    pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr kd_tree;
    void cube_made_by_plan(double biase = 0);
    void clear_cube(std::vector<Cube_namespace::cube_float_bin>::iterator &it);
    double cal_compont(void);

};

class Cube_float_super : public Cube_float
{
public:
    std::vector<Cube_namespace::cross_bin> cross_bin_vector;
    int cross_bin_num;

    Cube_float_super(double leng_th, int bin_num, int cross_bin_num);
    void cube_fit(double step_inc, int min_low_num);
    int Size_evau_x(void);
    void plan_intersection_points_cal(pcl::PointCloud<pcl::PointXYZINormal>::Ptr no_ground_points,
                                      pcl::PointCloud<pcl::PointXYZINormal>::Ptr target,
                                      int k_search, double radio_th);
    void clear_occupyed_cross_bin(void);
    int get_pointsNum_of_occupyed_bin(int cnt_input = 50);
    void get_intesPoints_cross_bin_vector_by_different_caverature(pcl::PointCloud<pcl::PointXYZINormal>::Ptr draw_points, int cnt_input=9999);
    void viewer_add_cube(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
private:

};

class Cube_cla
{
private:
    double Cube_len;
    int Cube_num_x;
    int Cube_num_y;
    int Cube_num_z;
    int Cube_numbs;

    double begin_x;
    double begin_y;
    double begin_z;

    double end_x;
    double end_y;
    double end_z;

public:
    std::vector<pcl::PointCloud<PointType>::Ptr> cube_vec;
    std::vector<Cube_namespace::id_stc> id_vec;
    std::vector<Cube_namespace::COMPONT_RADIO> compont_radio_vec;

    Cube_cla(double Cube_len, int Cube_num);
    void Put_points(pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr points_in);
    std::vector<int> get_id_vec(void);
    //std::vector<int> get_id_num_ground_vec(int);
    std::vector<Cube_namespace::id_stc> num_cube(int);
    std::vector<Cube_namespace::COMPONT_RADIO> get_compont_radio(double radio_th);
};

class Norm_est
{
private:
    pcl::search::KdTree<pcl::PointXYZINormal>::Ptr ne_tree;
    pcl::NormalEstimation<pcl::PointXYZINormal, pcl::Normal>::Ptr ne;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr points_temp;
    pcl::PointCloud<pcl::Normal>::Ptr normal_temp;
    double search_radio;
    int search_k;
public:
    Norm_est(double search_radio);
    Norm_est(int search_k);

    void Put_points(pcl::PointCloud<pcl::PointXYZINormal>::Ptr points_in,
                    pcl::PointCloud<pcl::Normal>::Ptr points_normal);
    void Put_points(pcl::PointCloud<pcl::PointXYZINormal>::Ptr points_in, pcl::PointCloud<pcl::PointXYZINormal>::Ptr points_out,
                              pcl::PointCloud<pcl::Normal>::Ptr points_normal);
    /*
    template <typename PointT>
    void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out, float thres);
     */
};

class Livox_feature
{
public:
    int CloudFeatureFlag[32000];

    Livox_feature();
    void extract_feature(pcl::PointCloud<PointType> &laserCloudIn,
                         pcl::PointCloud<PointType> &surfPointsFlat,
                         pcl::PointCloud<PointType> &cornerPointsSharp);
    void remove_sharp_in_ground(pcl::PointCloud<PointType> &cornerPointsSharp,
                                pcl::PointCloud<PointType> &groundPoints,
                                pcl::PointCloud<PointType> &cornerPointsSharpOut,
                                double radio, int k_th);
private:
    int scanID;
    int N_SCANS = 6;

    cv::Mat matA1;//(3, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matD1;//(1, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matV1;//(3, 3, CV_32F, cv::Scalar::all(0));

    bool plane_judge(const std::vector<PointType>& point_list,const int plane_threshold);

};

#endif