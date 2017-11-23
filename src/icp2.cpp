//// use the known transformation to get tf_transform 
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
  pcl::io::loadPCDFile("cylinder4.pcd", *cloud_in);
  std_msgs::Header header = pcl_conversions::fromPCL(cloud_in->header);


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
}
  return 0;
}
