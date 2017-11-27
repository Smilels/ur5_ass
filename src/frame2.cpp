#include<ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>

int main (int argc, char** argv)
{
  ros::init(argc, argv, "object_recognition");
  ros::NodeHandle nh;
  ros::Subscriber sub;
  tf::TransformBroadcaster *tf_pub1;
  tf_pub1 = new tf::TransformBroadcaster;
  tf::TransformBroadcaster *tf_pub2;
  tf_pub2 = new tf::TransformBroadcaster;
  ros::Publisher pub1= nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("pointcloud1", 1);
  ros::Publisher pub2= nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("pointcloud2", 1);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  pcl::PCDReader reader;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_tf (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tf (new pcl::PointCloud<pcl::PointXYZ>);
  //sensor_msgs::PointCloud2 pc;
  reader.read ("/homeL/demo/ws_ur/src/ur5_ass/src/pcd/rectangular2.pcd", *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl;

  Eigen::Matrix4f Tm;
  Tm <<     0.0714191,   -0.054278,   0.0441988,  0.00432322,
        -0.0409255,   -0.0836069,    0.036542,  0.00571429,
        0.0567861, 0.00800918,  -0.0819232,    0.122178,
            0,          0,          0,       1;
  tf::Vector3 origin;
  origin.setValue(static_cast<double>(Tm(0,3)),static_cast<double>(Tm(1,3)),static_cast<double>(Tm(2,3)));

  std::cout << Tm(0,3) <<Tm(1,3) <<Tm(2,3) << std::endl;
  tf::Matrix3x3 tf3d;
  tf3d.setValue(static_cast<double>(Tm(0,0)), static_cast<double>(Tm(0,1)), static_cast<double>(Tm(0,2)),
  static_cast<double>(Tm(1,0)), static_cast<double>(Tm(1,1)), static_cast<double>(Tm(1,2)),
  static_cast<double>(Tm(2,0)), static_cast<double>(Tm(2,1)), static_cast<double>(Tm(2,2)));

  tf::Quaternion tfqt;
  tf3d.getRotation(tfqt);
  geometry_msgs::Quaternion geoqt;
  tf::quaternionTFToMsg(tfqt,geoqt);
  ROS_INFO_STREAM("geoqt.x:"<<geoqt.x<<"geoqt.y:"<<geoqt.y<<"geoqt.z:"<<geoqt.z<<"geoqt.w:"<<geoqt.w);
 tf::Transform transform;
 transform.setOrigin(origin);
 transform.setRotation(tfqt);
  while (ros::ok())
  {
   cloud->header.frame_id="table_top";
   pub1.publish(cloud);
   //std::cout << "PointCloud frame " << cloud->header.frame_id << std::endl;

  tf_pub1->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/table_top", "/object_model"));
  //based on table_top, transform cloud to cloud_tf. Can't let cloud_tf frame equals to object_model otherwise cloud will tranfer twice.
  pcl_ros::transformPointCloud (*cloud, *cloud_tf, transform);
  //cloud_tf=cloud;
  //cloud_tf->header.frame_id="/object_model";
  pub2.publish(cloud_tf);
  Eigen::Vector4f min_pt;
  Eigen::Vector4f max_pt;
  pcl::getMinMax3D (*cloud_tf, min_pt, max_pt);
  Eigen::Vector4f center = (max_pt - min_pt)/2 + min_pt;
  Eigen::Vector4f shape=max_pt - min_pt;
  visualization_msgs::Marker cylinder;
  cylinder.ns = "object";
  //cylinder.header.frame_id = "/table_top";
  cylinder.type = visualization_msgs::Marker::CYLINDER;
  cylinder.color.g = 1.0f;
  cylinder.color.a = 1.0;
  cylinder.header.frame_id = "object_model";
  cylinder.pose.orientation.w=1;
  cylinder.scale.x = shape[0];
  cylinder.scale.y = shape[1];
  cylinder.scale.z = shape[2];
  cylinder.pose.position.x = center[0];
  cylinder.pose.position.y = center[1];
  cylinder.pose.position.z = center[2];
  cylinder.action = visualization_msgs::Marker::ADD;
  marker_pub.publish(cylinder);
  }

  return (0);
}
