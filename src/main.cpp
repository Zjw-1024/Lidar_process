#include"Lidar_process_core.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_node");

    ros::NodeHandle nh;

    LidarProcessCore core(nh);
    return 0;
}