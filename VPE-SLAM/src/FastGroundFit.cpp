#include "utils/FastGroundFit.h"
// using eigen lib
#include <Eigen/Dense>
#include "utils/Entity_Cla.h"

using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using Eigen::VectorXf;

pcl::PointCloud<PointType>::Ptr g_seeds_pc(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr g_ground_pc(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr g_not_ground_pc(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr g_all_pc(new pcl::PointCloud<PointType>());

/*
    @brief Compare function to sort points. Here use z axis.
    @return z-axis accent
*/
bool point_cmp(PointType a, PointType b){
    return a.z<b.z;
}
/*
    @brief Constructor of GPF Node.
    @return void
*/
GroundPlaneFit::GroundPlaneFit() {
    sensor_model_ = 32;
    sensor_height_ = 0.10;
    num_seg_ = 1;
    num_iter_ = 3;
    num_lpr_ = 500;
    num_lpr_dig = 0.05;
    th_seeds_ = 1.0;
    th_dist_ = 0.12;
}
/*
GroundPlaneFit::GroundPlaneFit(ros::NodeHandle &node_handle):node_handle_(node_handle){
    // Init ROS related
    ROS_INFO("GroundFit: Inititalizing Ground Plane Fitter...");
    node_handle_.param("/zy/sensor_model", sensor_model_, 32);
    ROS_INFO("GroundFit: Sensor Model: %d", sensor_model_);

    node_handle_.param("/zy/sensor_height", sensor_height_, 0.10);
    ROS_INFO("GroundFit: Sensor Height: %f", sensor_height_);

    node_handle_.param("/zy/num_seg", num_seg_, 1);
    ROS_INFO("GroundFit: Num of Segments: %d", num_seg_);

    node_handle_.param("/zy/num_iter", num_iter_, 3);
    ROS_INFO("GroundFit: Num of Iteration: %d", num_iter_);

    node_handle_.param("/zy/num_lpr", num_lpr_, 500);
    ROS_INFO("GroundFit: Num of LPR: %d", num_lpr_);

    node_handle_.param("/zy/num_lpr", num_lpr_dig, 0.05);
    ROS_INFO("GroundFit: Num of LPR: %f", num_lpr_dig);

    node_handle_.param("/zy/th_seeds", th_seeds_, 1.0);
    ROS_INFO("GroundFit: Seeds Threshold: %f", th_seeds_);

    node_handle_.param("/zy/th_dist", th_dist_, 0.12);
    ROS_INFO("GroundFit: Distance Threshold: %f", th_dist_);
}
 */
/*
    @brief The function to estimate plane model. The
    model parameter `normal_` and `d_`, and `th_dist_d_`
    is set here.
    The main step is performed SVD(UAV) on covariance matrix.
    Taking the sigular vector in U matrix according to the smallest
    sigular value in A, as the `normal_`. `d_` is then calculated
    according to mean ground points.

    @param g_ground_pc:global ground pointcloud ptr.

*/
void GroundPlaneFit::estimate_plane_(void){
    // Create covarian matrix in single pass.
    // TODO: compare the efficiency.
    Eigen::Matrix3f cov;
    Eigen::Vector4f pc_mean;
    pcl::computeMeanAndCovarianceMatrix(*g_ground_pc, cov, pc_mean);
    // Singular Value Decomposition: SVD
    JacobiSVD<MatrixXf> svd(cov,Eigen::DecompositionOptions::ComputeFullU);
    // use the least singular vector as normal
    normal_ = (svd.matrixU().col(2));
    // mean ground seeds value
    Eigen::Vector3f seeds_mean = pc_mean.head<3>();

    // according to normal.T*[x,y,z] = -d
    d_ = -(normal_.transpose()*seeds_mean)(0,0);
    // set distance threhold to `th_dist - d`
    th_dist_d_ = th_dist_ - d_;

    // return the equation parameters
}


/*
    @brief Extract initial seeds of the given pointcloud sorted segment.
    This function filter ground seeds points accoring to heigt.
    This function will set the `g_ground_pc` to `g_seed_pc`.
    @param p_sorted: sorted pointcloud

    @param ::num_lpr_: num of LPR points
    @param ::th_seeds_: threshold distance of seeds
    @param ::

*/
void GroundPlaneFit::extract_initial_seeds_(const pcl::PointCloud<PointType>& p_sorted){
    // LPR is the mean of low point representative
    double sum = 0;
    int cnt = 0;
    // Calculate the mean height value.
    for(int i=0;i<p_sorted.points.size() && cnt<num_lpr_dig * double(p_sorted.points.size());i++){
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double lpr_height = cnt!=0?sum/cnt:0;// in case divide by 0
    //std::cout << "lpr_height: " << lpr_height << endl;
    g_seeds_pc->clear();
    // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
    for(int i=0;i<p_sorted.points.size();i++){
        if(p_sorted.points[i].z < lpr_height + th_seeds_){
            g_seeds_pc->points.push_back(p_sorted.points[i]);
        }
    }
    // return seeds points
}

/*
    @brief Velodyne pointcloud callback function. The main GPF pipeline is here.
    PointCloud SensorMsg -> Pointcloud -> z-value sorted Pointcloud
    ->error points removal -> extract ground seeds -> ground plane fit mainloop
*/
std::vector<int> GroundPlaneFit::put_pointsCloud(pcl::PointCloud<PointType>::Ptr in_cloud)
{
    //std::cout << "inside in_cloud size: " << in_cloud->points.size() << endl;
    std::vector<int> ground_points_index;
    // 1.Msg to pointcloud
    pcl::PointCloud<PointType> laserCloudIn;
    pcl::PointCloud<PointType> laserCloudIn_org;

    pcl::copyPointCloud(*in_cloud, laserCloudIn);
    pcl::copyPointCloud(*in_cloud, laserCloudIn_org);

    //std::cout << "inside laserCloudIn: " << laserCloudIn.points.size() << endl;


// For mark ground points and hold all points
    g_all_pc->points.clear();
    PointType point;
    for(size_t i=0;i<laserCloudIn.points.size();i++){
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;
        point.intensity = 0U;
        g_all_pc->points.push_back(point);
    }

    //std::vector<int> indices;
    //pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn,indices);
    // 2.Sort on Z-axis value.
    sort(laserCloudIn.points.begin(),laserCloudIn.end(),point_cmp);
    // 3.Error point removal
    // As there are some error mirror reflection under the ground,
    // here regardless point under 2* sensor_height
    // Sort point according to height, here uses z-axis in default
    pcl::PointCloud<PointType>::iterator it = laserCloudIn.points.begin();
    //std::cout << "lase" << laserCloudIn.points[100].z << " " << laserCloudIn.points[1000].z << endl;
    for(int i=0;i<laserCloudIn.points.size();i++){
        if(laserCloudIn.points[i].z < -sensor_height_){
            it++;
        }else{
            break;
        }
    }

    //std::cout << "inside: laserCloudIn_org size: " << laserCloudIn.size() << endl;
    laserCloudIn.points.erase(it, laserCloudIn.points.end());
    //std::cout << "inside laserCloudIn after erase: " << laserCloudIn.points.size() << endl;

    // 4. Extract init ground seeds.
    extract_initial_seeds_(laserCloudIn);
    g_ground_pc = g_seeds_pc;

    //std::cout << "g_ground_pc size: " << g_ground_pc->points.size() << endl;
    // 5. Ground plane fitter mainloop
    for(int i=0;i<num_iter_;i++){
        estimate_plane_();
        g_ground_pc->clear();
        g_not_ground_pc->clear();

        //pointcloud to matrix
        MatrixXf points(laserCloudIn_org.points.size(),3);
       // std::cout << "MatrixXf size: " << points.size() << endl;
        int j =0;
        for(auto p:laserCloudIn_org.points){
            points.row(j++)<<p.x,p.y,p.z;
        }
        // ground plane model
        VectorXf result = points*normal_;
        // threshold filter
        for(int r=0;r<result.rows();r++){
            if(result[r]<th_dist_d_){
                g_all_pc->points[r].intensity= 200u;// means ground
                g_ground_pc->points.push_back(laserCloudIn_org[r]);
                in_cloud->points[r].curvature = Cube_namespace::GROUND;
            }else{
                g_all_pc->points[r].intensity= 150u;// means not ground and non clusterred
                g_not_ground_pc->points.push_back(laserCloudIn_org[r]);
                in_cloud->points[r].curvature = Cube_namespace::NO_GROUND;
            }
        }

    }
    /*
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZINormal> single_color(g_all_pc, "intensity");//点云颜色设置
    viewer->updatePointCloud<pcl::PointXYZINormal>(g_all_pc, single_color,"zy_cloud");
    */

        int cnt = 0;
        for (pcl::PointCloud<PointType>::iterator it_p = g_all_pc->begin(); it_p < g_all_pc->end(); ++it_p)
        {
            if (it_p->intensity == 150u)
            {
                ground_points_index.push_back(cnt);
            }
            cnt++;
        }
        return ground_points_index;
}

void GroundPlaneFit::fitlerGroundWithNormal(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_in, Eigen::Vector3d ref_vec, double th) {

    Eigen::Vector3d temp_vec;
    for (pcl::PointCloud<pcl::PointXYZINormal>::iterator it = cloud_in->begin(); it < cloud_in->end(); it++)
    {
       temp_vec << it->normal_x, it->normal_y, it->normal_z;
       if (it->curvature == Cube_namespace::GROUND)
       {
           if (temp_vec.cross(ref_vec).norm() > th)
           {
              it->curvature = Cube_namespace::NO_GROUND;
           }
       }
    }
}
void GroundPlaneFit::fitlerTopWithNormal(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_in, Eigen::Vector3d ref_vec, double th)
{
    Eigen::Vector3d temp_vec;
    for (pcl::PointCloud<pcl::PointXYZINormal>::iterator it = cloud_in->begin(); it < cloud_in->end(); it++)
    {
        temp_vec << it->normal_x, it->normal_y, it->normal_z;
        if (it->curvature != Cube_namespace::GROUND)
        {
            if (temp_vec.cross(ref_vec).norm() < th)
            {
                it->curvature = Cube_namespace::TOP;
            }
        }
    }
}