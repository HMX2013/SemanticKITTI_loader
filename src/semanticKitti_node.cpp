// For disable PCL complile lib, to use PointXYZILID
#define PCL_NO_PRECOMPILE

#include <visualization_msgs/Marker.h>
#include "tools/kitti_loader.hpp"
#include <signal.h>

using PointType = PointXYZILID;

ros::Publisher CloudPublisher;

std::string output_filename;
std::string acc_filename;
std::string pcd_savepath;


std::string data_dir;
string      algorithm;
string      seq;
double         fps;
bool        save_flag;

void signal_callback_handler(int signum) {
    cout << "Caught Ctrl + c " << endl;
    // Terminate program
    exit(signum);
}

template<typename T>
sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, std::string frame_id = "map") {
    sensor_msgs::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header.frame_id = frame_id;
    return cloud_ROS;
}

int main(int argc, char**argv) {
    ros::init(argc, argv, "Ros-Kitti-Publisher");

    ros::NodeHandle nh;
    nh.param<string>("/seq", seq, "00");
    nh.param<string>("/data_dir", data_dir, "/");
    nh.param<double>("/fps", fps, 1.0);

    ros::Rate r(fps);
    
    ros::Publisher CloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("/semi_kitti/non_ground_pc", 100, true);
    ros::Publisher GroundPublisher = nh.advertise<sensor_msgs::PointCloud2>("/semi_kitti/ground_pc", 100, true);
    ros::Publisher RawPcPublisher = nh.advertise<sensor_msgs::PointCloud2>("/semi_kitti/raw_pc", 100, true);

    signal(SIGINT, signal_callback_handler);

    std::string data_path = data_dir + "/" + seq;

    KittiLoader loader(data_path);
    int N = loader.size();

    std::cerr << "\033[1;32m[Kitti Publisher] Total " << N << " clouds are loaded\033[0m" << std::endl;
    for (int n = 0; n < N; ++n) {
        cout << "seq " << n << " is published!" << endl;
        pcl::PointCloud<PointType> pc_non_ground;
        pcl::PointCloud<PointType> pc_ground;
        pcl::PointCloud<PointType> pc_raw;
        loader.get_cloud(n, pc_non_ground, pc_ground, pc_raw);

        CloudPublisher.publish(cloud2msg(pc_non_ground));
        GroundPublisher.publish(cloud2msg(pc_ground));
        RawPcPublisher.publish(cloud2msg(pc_raw));
        r.sleep();
    }

    return 0;
}