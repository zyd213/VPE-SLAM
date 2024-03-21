#ifndef _LIDAR_ODOMETRY
#define _LIDAR_ODOMETRY
#include "utils/common.h"
#include "utils/math_tools.h"
#include "utils/timer.h"
#include "utils/LidarKeyframeFactor.h"

class LidarOdometry {
public:
    pcl::PointCloud<PointType>::Ptr edge_features;
    pcl::PointCloud<PointType>::Ptr surf_features;
    pcl::PointCloud<PointType>::Ptr full_cloud;

    pcl::PointCloud<PointType>::Ptr surf_last_ds;
    pcl::PointCloud<PointType>::Ptr edge_last_ds;


    pcl::PointCloud<PointType>::Ptr global_cloud_frame;
    pcl::PointCloud<PointType>::Ptr edge_cloud_frame;
    pcl::PointCloud<PointType>::Ptr surf_cloud_frame;

    pcl::KdTreeFLANN<PointType >::Ptr kd_tree_surf_last;
    pcl::KdTreeFLANN<PointType >::Ptr kd_tree_edge_last;

//  PointPseinfo   x   y    z     qw    qx   qy   qz
    pcl::PointCloud<PointPoseInfo>::Ptr pose_info_cloud_frame; //pose of each frame
    pcl::PointCloud<PointXYZI>::Ptr pose_cloud_frame; //position of each frame

    vector<pcl::PointCloud<PointType>::Ptr> surf_frames;
    deque<pcl::PointCloud<PointType>::Ptr> recent_surf_frames;

    vector<pcl::PointCloud<PointType>::Ptr> edge_frames;
    deque<pcl::PointCloud<PointType>::Ptr> recent_edge_frames;

    pcl::PointCloud<PointType>::Ptr surf_from_map;
    pcl::PointCloud<PointType>::Ptr surf_from_map_ds;

    pcl::PointCloud<PointType>::Ptr edge_from_map;
    pcl::PointCloud<PointType>::Ptr edge_from_map_ds;

    pcl::PointCloud<PointType>::Ptr surf_current_pts;
    pcl::PointCloud<PointType>::Ptr surf_normal;

    pcl::PointCloud<PointType>::Ptr edge_cur_pts;
    pcl::PointCloud<PointType>::Ptr edge_match_j;
    pcl::PointCloud<PointType>::Ptr edge_match_l;


    pcl::VoxelGrid<PointType> down_size_filter_surf;
    pcl::VoxelGrid<PointType> down_size_filter_surf_map;
    pcl::VoxelGrid<PointType> down_global_frame_cloud_filter;
    // pose representation: [quaternion: w, x, y, z | transition: x, y, z]
    double abs_pose[7];   //absolute pose from current frame to the first frame
    double rel_pose[7];   //relative pose between two frames

    int Local_frame_num = 20;
    bool system_initialized;

    int surf_res_cnt;
    int edge_res_cnt;

    int max_num_iter;
    int scan_match_cnt;

    int latest_frame_idx;

    bool kf = true;
    int kf_num = 0;

    Eigen::Vector3d trans_last_kf = Eigen::Vector3d::Zero();
    Eigen::Quaterniond quat_last_kF = Eigen::Quaterniond::Identity();

    bool if_to_deskew;
    double runtime = 0;

public:
    LidarOdometry();

    ~LidarOdometry(){}

    void allocateMemory();

    void initializeParameters();

    void undistortion(const pcl::PointCloud<PointType>::Ptr &pcloud, const Eigen::Vector3d trans, const Eigen::Quaterniond quat);

    void checkInitialization();

    void transformPoint(PointType const *const pi, PointType *const po);

    pcl::PointCloud<PointType>::Ptr
    transformCloud(const pcl::PointCloud<PointType>::Ptr &cloudIn, PointPoseInfo * PointInfoIn);

    void buildLocalMap();

    void clearCloud();

    void downSampleCloud();

    void savePoses();

    void findCorrespondingSurfFeatures();
    void findCorrespondingCornerFeatures();

    void poseInitialization();

    void computeRelative();

    void updateTransformationfusion();

    void updateTransformationsurfs();

    void updateTransformationedges();

};


#endif