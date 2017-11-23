//use two generated point cloud and do icp then transfer the transformation matrix to the tf_transform
//and use the transfer the transformation matrix to object pose
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>


int main (int argc, char** argv)
{
  ros::init(argc, argv, "icp");
  ros::NodeHandle nh;
  ros::Publisher marker_pub;
  tf::TransformBroadcaster tf_pub;
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

  cloud_in->width    = 5;
  cloud_in->height   = 1;
  cloud_in->is_dense = false;
  cloud_in->points.resize (cloud_in->width * cloud_in->height);
  for (size_t i = 0; i < cloud_in->points.size (); ++i)
  {
    cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }
  std::cout << "Saved " << cloud_in->points.size () << " data points to input:"
      << std::endl;
  *cloud_out = *cloud_in;
  std::cout << "size:" << cloud_out->points.size() << std::endl;


  for (size_t i = 0; i < cloud_in->points.size (); ++i)
    cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
  std::cout << "Transformed " << cloud_in->points.size () << " data points:"
      << std::endl;

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputCloud(cloud_in);
  icp.setInputTarget(cloud_out);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);

  std::cout << icp.getFinalTransformation() << std::endl;

  Eigen::Matrix4f Tm;
  Tm= icp.getFinalTransformation();
  tf::Vector3 origin;
  origin.setValue(static_cast<double>(Tm(0,3)),static_cast<double>(Tm(1,3)),static_cast<double>(Tm(2,3)));
 //std::cout << Tm(0,3) <<Tm(1,3) <<Tm(2,3) << std::endl;


  tf::Matrix3x3 tf3d;
  tf3d.setValue(static_cast<double>(Tm(0,0)), static_cast<double>(Tm(0,1)), static_cast<double>(Tm(0,2)),
        static_cast<double>(Tm(1,0)), static_cast<double>(Tm(1,1)),static_cast<double>(Tm(1,2)),
        static_cast<double>(Tm(2,0)), static_cast<double>(Tm(2,1)),static_cast<double>(Tm(2,2)));

  tf::Quaternion tfqt;
  geometry_msgs::Quaternion geoqt;
  tf3d.getRotation(tfqt);
  tf::quaternionTFToMsg(tfqt,geoqt);
  ROS_INFO_STREAM("geoqt.x"<<geoqt.x<<geoqt.y<<geoqt.z<<geoqt.w);
  tf::Transform transform;
  transform.setOrigin(origin);
  transform.setRotation(tfqt);
  while (ros::ok())
  {tf_pub.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "table_top", "/myfirst"));
  //ROS_INFO_STREAM("tfqt.x"<<transform.getRotation ());
//std::cout << tfqt(0,1) << std::endl;
  visualization_msgs::Marker cylinder;
  cylinder.ns="object";
  cylinder.header.frame_id = "table_top";
  cylinder.type = visualization_msgs::Marker::CYLINDER;
  cylinder.color.g = 1.0f;
  cylinder.color.a = 2.0;

  cylinder.pose.orientation=geoqt;
  cylinder.scale.x = 0.078;
  cylinder.scale.y = 0.078;
  cylinder.scale.z = Tm(2,3)*2000000;
  cylinder.pose.position.x = Tm(0,3);
  cylinder.pose.position.y = Tm(1,3);
  cylinder.pose.position.z = Tm(2,3);
  cylinder.action = visualization_msgs::Marker::ADD;
  marker_pub.publish(cylinder);}

  ros::spin();
  return (0);
}
