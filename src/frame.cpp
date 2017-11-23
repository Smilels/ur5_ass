#include<ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

int main (int argc, char** argv)
{
  ros::init(argc, argv, "object_recognition");
  ros::NodeHandle nh;
  ros::Subscriber sub;
  tf::TransformListener *tf_listener;
  tf_listener = new tf::TransformListener;
  ros::Publisher pub1= nh.advertise<sensor_msgs::PointCloud2> ("pointcloud1", 1);
  ros::Publisher pub2= nh.advertise<sensor_msgs::PointCloud2> ("pointcloud2", 1);
  // Read in the cloud data
  pcl::PCDReader reader;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_tf (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2),cloud_tf (new pcl::PCLPointCloud2);
  sensor_msgs::PointCloud2 pc;
  reader.read ("/homeL/demo/ws_ur/src/ur5_ass/src/asymmetry.pcd", *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->width * cloud->height << " data points." << std::endl;
  while (ros::ok())
  {pcl_conversions::fromPCL (*cloud, pc);
  pc.header.frame_id="table_top";
  pub1.publish(pc);

  sensor_msgs::PointCloud2 pc2;
  cloud->header.frame_id = "/world";
  pcl_conversions::fromPCL (*cloud, pc2);
  pub2.publish(pc2);}
  //std::cout << "PointCloud header: " << cloud_tf->header.frame_id << std::endl;

  ros::spin();
  return (0);
}
