#include "livox_ros_driver/CustomMsg.h"
#include "utils/common.h"

#define  PI  3.1415926535
#define Write_frame true
#define Frame_PATH "/home/adminpc/zy_ka/src/zy_ka/data/test_data300.pcd"
#define Frame_to_save 300

using namespace std;
#define LINE_NUM 6
ros::Publisher pub_ros_points;
ros::Publisher pub_line0_points;

uint32_t seq_cnt = 0;

string frame_id = "zy_ka";
std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> line_points;
void livoxLidarHandler(const livox_ros_driver::CustomMsgConstPtr& livox_msg_in) { //livox lidar point的发布频率大概是10ｈｚ
    pcl::PointCloud<PointXYZINormal> pcl_in;
    auto time_end = livox_msg_in->points.back().offset_time;
    for (unsigned int i = 0; i < livox_msg_in->point_num; ++i) {
        PointXYZINormal pt;
        pt.x = livox_msg_in->points[i].x;
        pt.y = livox_msg_in->points[i].y;
        pt.z = livox_msg_in->points[i].z;
        float s = float(livox_msg_in->points[i].offset_time / (float)time_end);
        pt.intensity = livox_msg_in->points[i].reflectivity;
        //pt.intensity = livox_msg_in->points[i].line + s*0.1; // integer part: line number; decimal part: timestamp
        //pt.curvature = 0.1 * livox_msg_in->points[i].reflectivity;
        pcl_in.push_back(pt);
        line_points[livox_msg_in->points[i].line]->push_back(pt);
    }

    /// timebase 5ms ~ 50000000, so 10 ~ 1ns

    ros::Time timestamp(livox_msg_in->header.stamp.toSec());

    sensor_msgs::PointCloud2 pcl_ros_msg;
    pcl::toROSMsg(pcl_in, pcl_ros_msg);
    pcl_ros_msg.header.stamp = timestamp;
    pcl_ros_msg.header.frame_id = frame_id;

    /***********pub lines zero*************/
    sensor_msgs::PointCloud2 pcl_ros_msg_line0;
    pcl::toROSMsg(*line_points[5], pcl_ros_msg_line0);
    pcl_ros_msg_line0.header.stamp = timestamp;
    pcl_ros_msg_line0.header.frame_id = frame_id;
    for (int i = 0; i < LINE_NUM; i++)
    {
        line_points[i]->clear();
    }
    /***************************************/
    pub_ros_points.publish(pcl_ros_msg);
    pub_line0_points.publish(pcl_ros_msg_line0);

    if (seq_cnt == Frame_to_save)
    {
        pcl::io::savePCDFile(Frame_PATH, pcl_in);
        std::cout << "saved! " << Frame_to_save << " frame-th! " << pcl_in.size() << endl;
    }
    seq_cnt++;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "FormatConvert");
    ros::NodeHandle nh;
    for (int i = 0; i < LINE_NUM; i++) //line 6
    {
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr temp_lines_points;
        temp_lines_points.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
       line_points.push_back(temp_lines_points);
    }
    ROS_INFO("\033[1;32m---->\033[0m CustomMsg Started.");

    if (!getParameter("/common/frame_id", frame_id))
    {
        ROS_WARN("frame_id not set, use default value: zy_ka");
        frame_id = "zy_ka";
    }

    ros::Subscriber sub_livox_lidar = nh.subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 100, livoxLidarHandler);

    pub_ros_points = nh.advertise<sensor_msgs::PointCloud2>("/CustomMsgViewer/Points", 100);
    pub_line0_points = nh.advertise<sensor_msgs::PointCloud2>("/zy/line0_points", 100);

    ros::spin();
}
