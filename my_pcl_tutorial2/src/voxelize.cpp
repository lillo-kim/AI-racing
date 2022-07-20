#include <iostream>
#include <cmath>
#include <vector>

// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

// PCL specific includes
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>


using namespace std;

// voxelized one
ros::Publisher pub;

void input (const sensor_msgs::PointCloud2ConstPtr& scan)
{
  //////Msg to pointcloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr src(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*scan, *src);

  pcl::PointCloud<pcl::PointXYZI>::Ptr voxelized(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;

  float voxel_size = 0.08f; // smaller -> more points, larger -> less points
  voxel_filter.setInputCloud (src);
  voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
  voxel_filter.filter(*voxelized);

  pcl::PCLPointCloud2 voxelized_cloud;
  pcl::toPCLPointCloud2(*voxelized, voxelized_cloud);
  sensor_msgs::PointCloud2 voxelized_output;
  pcl_conversions::fromPCL(voxelized_cloud, voxelized_output);

  voxelized_output.header.frame_id = "velodyne";
  pub.publish(voxelized_output);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "voxelize");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/to_voxel_node", 100, input);

  // Create a ROS publisher for the output point cloud
  // pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_points", 100);
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_points_voxelized", 100);

  // Spin
  ros::spin ();
}
