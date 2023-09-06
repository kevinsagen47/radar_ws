#include "opencv2/video/tracking.hpp"
#include "pcl_ros/point_cloud.h"
#include <algorithm>
#include <fstream>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <iterator>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <string.h>

#include <pcl/common/centroid.h>
#include <pcl/common/geometry.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <limits>
#include <utility>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "kf_tracker/point_types.h"
#include <pcl/point_types.h>
using namespace std;
using namespace cv;

//void cloud_cb(const pcl::PointCloud<radar_pcl::PointXYZIVR>& input){
void cloud_cb(const pcl::PointCloud<radar_pcl::PointXYZIVR>::ConstPtr& input){
    cout<<"entered"<<"\n";
    //std::cout << typeid(input).name() << '\n';

}

int main(int argc, char **argv) {
  // ROS init
  ros::init(argc, argv, "kf_tracker");
  ros::NodeHandle nh;
  std::string topic = nh.resolveName("point_types");

  // Publishers to publish the state of the objects (pos and vel)
  // objState1=nh.advertise<geometry_msgs::Twist> ("obj_1",1);

  cout << "About to setup callback\n";

  // Create a ROS subscriber for the input point cloud
  ///ros::Subscriber sub = nh.subscribe("filtered_cloud", 1, cloud_cb);
  ros::Subscriber sub = nh.subscribe<pcl::PointCloud<radar_pcl::PointXYZIVR>> ("filtered_cloud", 1, cloud_cb);
  ros::spin();
}

