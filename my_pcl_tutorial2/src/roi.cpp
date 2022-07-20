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

double ROI_theta(double x, double y);
using namespace std;

// Publish PCL data in a certain ROI
ros::Publisher pub;

double ROI_theta(double x, double y){
    double r;
    double theta;

    r = sqrt((x*x)+(y*y));
    theta = acos(x/r)*180/M_PI;
    return theta;
}

double distance(double x, double y){
  double r = sqrt((x*x)+(y*y));
  return r;
}

void input (const sensor_msgs::PointCloud2ConstPtr& scan)
{
  //////Msg to pointcloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr src(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*scan, *src);
  pcl::PointCloud<pcl::PointXYZI> src_cloud = *src;
  
  for(unsigned int i=0; i<src_cloud.points.size(); i++){
      if((ROI_theta(src_cloud.points[i].x, src_cloud.points[i].y) > 90) && (ROI_theta(src_cloud.points[i].x, src_cloud.points[i].y) < 270)){
          src_cloud.points[i].x = 0;
          src_cloud.points[i].y = 0;
          src_cloud.points[i].z = 0;
      }
      if(src_cloud.points[i].x < 0.0){
          src_cloud.points[i].x = 0;
          src_cloud.points[i].y = 0;
          src_cloud.points[i].z = 0;
      }
  }

  /*for(unsigned int i=0; i<src_cloud.points.size(); i++){
      if((distance(src_cloud.points[i].x, src_cloud.points[i].y) > 18)){
          src_cloud.points[i].x = 0;
          src_cloud.points[i].y = 0;
          src_cloud.points[i].z = 0;
      }
  }*/


  pcl::PCLPointCloud2 roied_cloud;
  pcl::toPCLPointCloud2(src_cloud, roied_cloud);
  sensor_msgs::PointCloud2 roied_output;
  pcl_conversions::fromPCL(roied_cloud, roied_output);

  roied_output.header.frame_id = "velodyne";
  pub.publish(roied_output);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "roi");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 100, input);
  // /velodyne_points
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_points_roied", 100);
  

  // Spin
  ros::spin ();
}
