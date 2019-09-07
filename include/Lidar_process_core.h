#include<vector>
#include <ros/ros.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <ctime>
#include <chrono>
#include <unordered_set>

class LidarProcessCore
{
private:

    ros::Publisher cloud_filtered_pub_;
    ros::Publisher obstCloud_pub_;
    ros::Publisher planeCloud_pub_;

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr> 
        SeparateClouds(pcl::PointIndices::Ptr inliers, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudinput);

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr> 
        SegmentPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceThreshold);
    
    void Lidar_Callback(const sensor_msgs::PointCloud2ConstPtr& input);

public:
    LidarProcessCore(ros::NodeHandle & nh);
    ~LidarProcessCore();
};