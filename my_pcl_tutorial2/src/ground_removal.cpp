#include <iostream>
#include <cmath>
#include <vector>
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;

// Publish PCL data in a certain ROI
ros::Publisher pub_obs;
ros::Publisher pub_plane;

std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    pcl::PointCloud<pcl::PointXYZI>::Ptr obstCloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr planeCloud (new pcl::PointCloud<pcl::PointXYZI>);

    // we are pushing back every point which has the index saved in "indices" into planeCloud
    for(int index : inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);

    // extracting object
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    // reference cloud
    extract.setInputCloud (cloud);
    // inliers
    extract.setIndices (inliers);
    extract.setNegative (true);
    // this way, all the points that are not the inliers are kept and it'll be assigned to obstCloud
    extract.filter (*obstCloud);

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}

void input (const sensor_msgs::PointCloud2ConstPtr& scan)
{
    // parameters
    int maxIterations = 1000;
    float distanceThreshold = 0.1;

    //////Msg to pointcloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr src (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*scan, *src);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold (distanceThreshold);
    seg.setInputCloud (src);
    seg.segment (*inliers, *coefficients);

    if(inliers->indices.size() == 0){
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult = SeparateClouds(inliers, src);

    pcl::PCLPointCloud2 obstacle_cloud, plane_cloud;
    
    pcl::toPCLPointCloud2(*(segResult.first), obstacle_cloud);
    pcl::toPCLPointCloud2(*(segResult.second), plane_cloud);
    
    sensor_msgs::PointCloud2 obstacle_output;
    sensor_msgs::PointCloud2 plane_output;
    
    pcl_conversions::fromPCL(obstacle_cloud, obstacle_output);
    pcl_conversions::fromPCL(plane_cloud, plane_output);

    obstacle_output.header.frame_id = "velodyne";
    plane_output.header.frame_id = "velodyne";

    pub_obs.publish(obstacle_output);
    pub_plane.publish(plane_output);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "ground_removal");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points_voxelized", 100, input);
  // /velodyne_points
  // Create a ROS publisher for the output point cloud
  pub_obs = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_points_obstacles", 100);
  pub_plane = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_points_plane", 100);

  // Spin
  ros::spin ();
}
