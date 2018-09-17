#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Bool.h>
#include <points_msgs/plane_coefficients.h>
#include <iostream>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>
#include <pcl/features/fpfh_omp.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <pcl/common/common.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>


std_msgs::Header _velodyne_header;
ros::Publisher _pub_ground_cloud;
ros::Publisher _pub_plane_coefficients;


using namespace std;

void publishCloud(const ros::Publisher* in_publisher, const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr)
{
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
  cloud_msg.header=_velodyne_header;
  in_publisher->publish(cloud_msg);
}


void segCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr)
{
  float in_max_height = 0.2;
  float in_floor_max_angle = 0.1;

  pcl::SACSegmentation<pcl::PointXYZI> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setAxis(Eigen::Vector3f(0, 0, 1));
  seg.setEpsAngle(in_floor_max_angle);

  seg.setDistanceThreshold(in_max_height);  // floor distance
  seg.setOptimizeCoefficients(true);
  seg.setInputCloud(in_cloud_ptr);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0)
  {
    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
  }

  // REMOVE THE FLOOR FROM THE CLOUD
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(in_cloud_ptr);
  extract.setIndices(inliers);
  // EXTRACT THE FLOOR FROM THE CLOUD
  extract.setNegative(false);  // true removes the indices, false leaves only the indices
  extract.filter(*out_cloud_ptr);

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  points_msgs::plane_coefficients msg;
  msg.a = coefficients->values[0];
  msg.b = coefficients->values[1];
  msg.c = coefficients->values[2];
  msg.d = coefficients->values[3];

  _pub_plane_coefficients.publish(msg);
}


void velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& in_sensor_cloud)
{
  
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_sensor_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    _velodyne_header = in_sensor_cloud->header;

    pcl::fromROSMsg(*in_sensor_cloud, *current_sensor_cloud_ptr);

    segCloud(current_sensor_cloud_ptr, ground_cloud_ptr);

    publishCloud(&_pub_ground_cloud, ground_cloud_ptr);


}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "curb_based_segmentation");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe ("/points_raw", 1, velodyne_callback);

  _pub_ground_cloud = nh.advertise<sensor_msgs::PointCloud2>("/ground_points",1);
  _pub_plane_coefficients = nh.advertise<points_msgs::plane_coefficients>("/plane_coefficients",1);
 
  ros::spin ();
}