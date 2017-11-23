//// use the known transformation to get the pose and tf_transform
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

  Eigen::Matrix4f Tm;
  Tm <<     0.714191,   -0.54278,   0.441988,  0.0432322,
        0.409255,   0.836069,    0.36542,  0.0571429,
        -0.567861, -0.0800918,   0.819232,    1.22178,
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
  while  (ros::ok())
   {tf_pub.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/table_top", "/lifting_eye"));
  visualization_msgs::Marker cylinder;
  cylinder.ns="object1";
  cylinder.header.frame_id = "/table_top";
  cylinder.type = visualization_msgs::Marker::CYLINDER;
  cylinder.color.g = 1.0f;
  cylinder.color.a = 2.0;

  cylinder.pose.orientation=geoqt ;
  cylinder.scale.x = 0.078;
  cylinder.scale.y = 0.078;
  cylinder.scale.z = Tm(2,3)/2;
  cylinder.pose.position.x = Tm(0,3);
  cylinder.pose.position.y = Tm(1,3);
  cylinder.pose.position.z = Tm(2,3);
  cylinder.action = visualization_msgs::Marker::ADD;
  marker_pub.publish(cylinder);}
  return 0;
}
