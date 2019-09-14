#include"Lidar_process_core.h"

LidarProcessCore::LidarProcessCore(ros::NodeHandle &nh)
{

    std::string lidar_topic;
    ros::param::get("lidar_topic", lidar_topic);
     //ros::Subscriber sub = nh.subscribe("/velodyne_points", 10, Lidar_Callback);
    ros::Subscriber sub = nh.subscribe(lidar_topic, 5, &LidarProcessCore::Lidar_Callback,this);  

    cloud_filtered_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 3);
    obstCloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/obstCloud", 3);
    //planeCloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/planeCloud", 3);
    ros::spin();
}

LidarProcessCore::~LidarProcessCore(){}

std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr> 
    LidarProcessCore::SeparateClouds(pcl::PointIndices::Ptr inliers, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    pcl::PointCloud<pcl::PointXYZI>::Ptr obstCloud (new pcl::PointCloud<pcl::PointXYZI> ());
    pcl::PointCloud<pcl::PointXYZI>::Ptr planeCloud (new pcl::PointCloud<pcl::PointXYZI> ());

    for(int index : inliers -> indices)
        planeCloud->points.push_back(cloud->points[index]);

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr>
       segResult(obstCloud, planeCloud);
    return segResult;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LidarProcessCore::Cloud_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudinput)
{
  auto startTime = std::chrono::steady_clock::now();
  
  double leaf_size;
  ros::param::get("leaf_size", leaf_size);
 
  //pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in {new pcl::PointCloud<pcl::PointXYZI>};
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_final {new pcl::PointCloud<pcl::PointXYZI>};

  //sensor_msgs::PointCloud2 cloud_out;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_VG_filtered {new pcl::PointCloud<pcl::PointXYZI>};
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_CB_filtered {new pcl::PointCloud<pcl::PointXYZI>};

  //pcl::fromROSMsg(*input,*cloud_in);

  pcl::VoxelGrid<pcl::PointXYZI> sor; 
  sor.setInputCloud (cloudinput);     
  sor.setLeafSize (leaf_size, leaf_size, leaf_size);
  sor.filter(*cloud_VG_filtered); 

  pcl::CropBox<pcl::PointXYZI> region(true);
  region.setMin(Eigen::Vector4f (0, -10, -0.5, 1));
  region.setMax(Eigen::Vector4f (100, 10, 2, 1));
  region.setInputCloud(cloud_VG_filtered);
  region.filter(*cloud_CB_filtered);

  // std::vector<int> indices;

  // pcl::CropBox<pcl::PointXYZI> roof(true);
  // roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
  // roof.setMax(Eigen::Vector4f(2.6,1.7,-0.4,1));
  // roof.setInputCloud(cloud_CB_filtered);
  // roof.filter(indices);

  // pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
  // for(int point:indices)
  //     inliers->indices.push_back(point);
  // pcl::ExtractIndices<pcl::PointXYZI> extract;
  // extract.setInputCloud(cloud_CB_filtered);
  // extract.setIndices(inliers);
  // extract.setNegative(true);
  // extract.filter(*cloud_final);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

  ROS_INFO("filtering took %ld milliseconds",elapsedTime.count() );

  //return cloud_final;
  return cloud_CB_filtered;


}

std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr> 
    LidarProcessCore::SegmentPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceThreshold)
{
  ROS_INFO("segmentation process");
    // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.

  std::unordered_set<int> inlierResult;
  for(int i=0;i<maxIterations;++i)
	{
		// Randomly sample subset and fit line
		int first;
		int second;
		int third;
		std::unordered_set<int> inlier;
		do
		{
			first = rand()%(cloud->size());
			second = rand()%(cloud->size());
			third = rand()%(cloud->size());
		}while(first==second || second == third || third == first);
		std::vector<float> v1,v2;
		v1.push_back(cloud->points[second].x - cloud->points[first].x);
		v1.push_back(cloud->points[second].y - cloud->points[first].y);
		v1.push_back(cloud->points[second].z - cloud->points[first].z);

		v2.push_back(cloud->points[third].x - cloud->points[first].x);
		v2.push_back(cloud->points[third].y - cloud->points[first].y);
		v2.push_back(cloud->points[third].z - cloud->points[first].z);


		auto a = v1[1]*v2[2] - v1[2]*v2[1];
		auto b = v1[2]*v2[0] - v1[0]*v2[2];
		auto c = v1[0]*v2[1] - v1[1]*v2[0];
		auto d = -(a*cloud->points[first].x+b*cloud->points[first].y+c*cloud->points[first].z);
    //平面方程a*x+b*y+c*z+d=0 法向量（a,b,c）
		// Measure distance between every point and fitted line
		for(int j=0;j<cloud->size();j++)
		{
			pcl::PointXYZI point = cloud->points[j];
			auto distance = fabs(a*point.x+b*point.y+c*point.z+d)/(sqrt(a*a+b*b+c*c));
			if(distance<=distanceThreshold)
			{
				inlier.insert(j);
			}
		}
		// If distance is smaller than threshold count it as inlier
		if(inlier.size()>inlierResult.size())
		    inlierResult = inlier;
	}

  pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
  for(int tmp : inlierResult)
    inliers->indices.push_back(tmp);
  if(inliers->indices.size()==0)
  {
    ROS_WARN("could not estimate a planar model for the given dataset.");
  }

  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> 
    segResult = SeparateClouds(inliers,cloud);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  ROS_INFO("plane segmentation took %ld milliseconds",elapsedTime.count());

  return segResult;
}

void LidarProcessCore::Lidar_Callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  //ROS_INFO("received lidar message");

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in {new pcl::PointCloud<pcl::PointXYZI>};
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered {new pcl::PointCloud<pcl::PointXYZI>};

  pcl::fromROSMsg(*input,*cloud_in);
  ROS_INFO("cloud_in count: %ld",cloud_in->points.size());

  //filter process
  cloud_filtered = Cloud_filter(cloud_in);
  ROS_INFO("cloud_filtered count: %ld",cloud_filtered->points.size());

  sensor_msgs::PointCloud2 cloud_filtered_out;
  pcl::toROSMsg(*cloud_filtered,cloud_filtered_out);
  cloud_filtered_pub_.publish(cloud_filtered_out); 

  //segmentation process
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr> segment
      = SegmentPlane(cloud_filtered,1000,0.2);
  
  ROS_INFO("obstCloud count %ld",segment.first->size());
  ROS_INFO("planeCloud count %ld",segment.second->size());

  sensor_msgs::PointCloud2 obstCloud_out;
  //sensor_msgs::PointCloud2 planeCloud_out;

  pcl::toROSMsg(*segment.first,obstCloud_out);
  //pcl::toROSMsg(*segment.second,planeCloud_out);

  obstCloud_pub_.publish(obstCloud_out);
  //planeCloud_pub_.publish(planeCloud_out);
}