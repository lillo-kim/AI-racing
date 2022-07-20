#include <iostream>
#include <cmath>
#include <vector>
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/impl/centroid.hpp>
#include <pcl/filters/statistical_outlier_removal.h> //outlier sorting

// from other packages
#include "object_builders/base_object_builder.hpp"
#include "object_builders/object_builder_manager.hpp"
#include "common/publisher.hpp"
#include "common/color.hpp"

#include "common/common.hpp"
#include "common/geometry.hpp"      // common::geometry::calcYaw4DirectionVector
#include "common/transform.hpp"     // common::transform::transformPointCloud
#include "common/types/object.hpp"  // ObjectPtr
#include "common/types/type.h"

using namespace std;
using namespace autosense;

struct Box
{
  float x_min;
  float y_min;
  float z_min;
  float x_max;
  float y_max;
  float z_max;
};

// Publish clustered PCD
ros::Publisher pub;
// Publish bounding boxes in the type of MarkerArray
ros::Publisher objects_pub_1;
// ros::Publisher objects_pub_2;
boost::shared_ptr<object_builder::BaseObjectBuilder> object_builder_;

void input (const sensor_msgs::PointCloud2ConstPtr& scan)
{
  //////Msg to pointcloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*scan, *cloud);
  std_msgs::Header header = scan->header;

///////////////////////clustering 시작///////////////////////

  float clusterTolerance = 1; // 0.53
  int minSize = 10;
  int maxSize = 500;
  float car_length = 1.75;
  float car_width = 1.05;
  float car_height = 0.822;
  float car_dis = 6;

  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  // cluter라고 판단할만한 point들의 거리
  ec.setClusterTolerance(clusterTolerance);
  // 한 cluster의 최소, 최대 포인트 개수
  ec.setMinClusterSize(minSize);
  ec.setMaxClusterSize(maxSize);
  // 검색 방법이 tree라는 것을 명시해주는 것
  ec.setSearchMethod(tree);
  // clustering 결과를 입력할 pointer 집어넣기. 현 source 파일 기준 cloud
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
  pcl::PointCloud<pcl::PointXYZI> cloud1;
  //std::cout << "Number of clusters is equal to " << cluster_indices.size () << std::endl;

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) // all cluster
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_filtered (new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_temp (new pcl::PointCloud<pcl::PointXYZI>);

      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){ // each cluster
        cluster->points.push_back(cloud->points[*pit]);
        cluster_temp->points.push_back(cloud->points[*pit]);
        cloud1.push_back(cloud->points[*pit]);
      }

      // get each index in cluster in cluster_indices and push it into the cloud above      
      /*pcl::PointXYZI minPoint, maxPoint;
      pcl::getMinMax3D(*cluster_filtered, minPoint, maxPoint);
      Box box;
      box.x_min = minPoint.x;
      box.y_min = minPoint.y;
      box.z_min = minPoint.z;
      box.x_max = maxPoint.x;
      box.y_max = maxPoint.y;
      box.z_max = maxPoint.z;
      float xsize = std::abs(box.x_max - box.x_min);
      float ysize = std::abs(box.y_max - box.y_min);
      float zsize = std::abs(box.z_max - box.z_min);
      
      if (xsize < 2.5 && ysize<1.5 && zsize >0.05){ //&& zsize >0.05
        cluster->width = cluster_filtered->points.size();
        cluster->height = 1;
        cluster->is_dense = true;
        clusters.push_back(cluster);
      }*/

      cluster->width = cluster->points.size();
      cluster->height = 1;
      cluster->is_dense = true;
      clusters.push_back(cluster);

      std::vector<autosense::ObjectPtr> temp_obj;
      object_builder_->build(clusters, &temp_obj);

      for (int i=0; i < temp_obj.size(); i++) {
        float box_len = temp_obj.at(i)->length;
        float box_width = temp_obj.at(i)->width;
        float box_height = temp_obj.at(i)->height;
        float box_x = temp_obj.at(i)->ground_center[0]; // box center x point
        float box_y = temp_obj.at(i)->ground_center[1];
        float box_distance = sqrt(pow(box_x, 2) + pow(box_y, 2));

        if(box_len * box_width > 10 || box_len * box_height > 10 || box_width * box_height >10 || box_y > 6 || box_y < -6) {
          clusters.pop_back();
          cout << "Too large : Box is deleted!" << endl;
        }

        else if (box_distance > car_dis && box_len > car_length && box_width > car_width && box_height > car_height ) {
          clusters.pop_back();
          
          pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor; // sorting
          sor.setInputCloud(cluster_temp);
          sor.setMeanK(30); // 분석시 고려할 이웃 점 수             
          sor.setStddevMulThresh(0.05); // outlier로 처리할 거리 정보
          sor.filter(*cluster_filtered);

          cluster_filtered->width = cluster_filtered->points.size();
          cluster_filtered->height = 1;
          cluster_filtered->is_dense = true;
          clusters.push_back(cluster_filtered);
          cout << "Statical second bounding box" << endl;
        }
        else
          cout << "First bounding box" << endl;          
      }
    }
  
  //auto csize=clusters.size ();
  //std::cout << "cluster size : " << csize << std::endl;

  std::vector<autosense::ObjectPtr> objects;
  object_builder_->build(clusters, &objects);
  // Make bounding boxes
  autosense::common::publishObjectsMarkers(objects_pub_1, header, autosense::common::MAGENTA.rgbA, objects);

  pcl::PCLPointCloud2 clustered_cloud;
  pcl::toPCLPointCloud2(cloud1, clustered_cloud);
  sensor_msgs::PointCloud2 clustered_output;
  pcl_conversions::fromPCL(clustered_cloud, clustered_output);

  clustered_output.header.frame_id = "velodyne";
  pub.publish(clustered_output);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cluster");
  ros::NodeHandle nh;

  object_builder_ = object_builder::createObjectBuilder();
  if (nullptr == object_builder_) {
      ROS_FATAL("Failed to create object_builder_.");
      return -1;
  }
  objects_pub_1 = nh.advertise<visualization_msgs::MarkerArray>("/box_bounded", 1);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/filtered_points_no_ground", 100, input);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/points_clustered", 100);
  

  // Spin
  ros::spin ();
}