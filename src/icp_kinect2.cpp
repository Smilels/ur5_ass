//use trure point cloud and model do icp then transfer the transformation matrix to the tf_transform
//and use the transfer the transformation matrix to object pose
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ur5_ass/ObjectPose.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>


typedef pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
typedef pcl::PointXYZRGB Point;
class ICPregister{
private:
  ros::NodeHandle nh;
  ros::Publisher marker_pub;
  ros::Subscriber sub;
  tf::TransformBroadcaster *tf_pub;
  tf::TransformListener *tf_listener;
  ros::Publisher pub1;
  ros::Publisher pub2;
  ros::Publisher pub3;
  ros::Publisher object_pose_pub;

public:
  ICPregister(){
  sub = nh.subscribe<point_cloud>("kinect2/qhd/points", 1, &ICPregister::pointCloudCb,this);
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  tf_pub=new tf::TransformBroadcaster;
  tf_listener = new tf::TransformListener;
  pub1= nh.advertise<point_cloud> ("pointcloud1", 1);
  pub2= nh.advertise<point_cloud> ("pointcloud2", 1);
  pub3= nh.advertise<point_cloud> ("pointcloud3", 1);
  object_pose_pub=nh.advertise<ur5_ass::ObjectPose> ("Tower/ObjectPose", 1);
}
  void pointCloudCb(const point_cloud::ConstPtr& cloud_in){
  point_cloud::Ptr cloud_tf (new point_cloud);
  point_cloud::Ptr cloud_filtered (new point_cloud);
  point_cloud::Ptr cloud_model (new point_cloud);
  std_msgs::Header header = pcl_conversions::fromPCL(cloud_in->header);
  visualization_msgs::Marker cylinder;
  cylinder.ns = "object";
  cylinder.header = header;
  cylinder.header.frame_id = "/table_top";
  cylinder.action = visualization_msgs::Marker::DELETE;
  cylinder.type = visualization_msgs::Marker::CYLINDER;
  cylinder.color.g = 1.0f;
  cylinder.color.a = 1.0;
  tf::StampedTransform transform;
  try{
      tf_listener->waitForTransform("/table_top", header.frame_id, header.stamp, ros::Duration(5.0));
      tf_listener->lookupTransform ("/table_top", header.frame_id, header.stamp, transform);
  }
  catch(std::runtime_error &e){
      return;
  }

  pcl_ros::transformPointCloud (*cloud_in, *cloud_tf, transform);
  cloud_tf->header.frame_id = "/table_top";

  pcl::CropBox<Point> box;
  box.setInputCloud(cloud_tf);
  //this is our region of interest
  box.setMin(Eigen::Vector4f(-0.45,-0.35,0.05,1.0));
  box.setMax(Eigen::Vector4f(0.5,0.15,0.5,1.0));
  box.filter (*cloud_filtered);

  std::vector<pcl::PointIndices> indices;
  pcl::EuclideanClusterExtraction<Point> cluster;
  cluster.setClusterTolerance (0.02);
  cluster.setMinClusterSize (10);
  cluster.setInputCloud(cloud_filtered);
  cluster.extract(indices);

  if(indices.size() == 0){
      marker_pub.publish(cylinder);
      return;
  }

  pcl::ExtractIndices<Point> extractor;
  pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices(indices[0]));
  extractor.setInputCloud(cloud_filtered);
  extractor.setIndices(objectIndices);
  point_cloud::Ptr objectCloud(new point_cloud);
  extractor.filter(*objectCloud);//object point cloud
  pub1.publish(objectCloud);

  point_cloud::Ptr cloud_target(new point_cloud);
  pcl::PCDReader reader;
  reader.read ("/homeL/demo/ls_ws/src/ur5_ass/src/pcd/cylinder4.pcd", *cloud_target);
  std::cout << "PointCloud before filtering has: " << cloud_target->points.size () << " data points." << std::endl;
  cloud_target->header.frame_id="table_top";
  Eigen::Vector4f min_pt;
  Eigen::Vector4f max_pt;
  pcl::getMinMax3D (*cloud_target, min_pt, max_pt);
  Eigen::Vector4f center = (max_pt - min_pt)/2 + min_pt;
  Eigen::Vector4f shape=max_pt - min_pt;
  pub2.publish(cloud_target);

  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  icp.setInputCloud(objectCloud);
  icp.setInputTarget(cloud_target);
  point_cloud Final;
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
  transform.setOrigin(origin);
  transform.setRotation(tfqt);
  tf_pub->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "table_top", "/object_model"));
  //ROS_INFO_STREAM("tfqt.x"<<transform.getRotation ());
  //std::cout << tfqt(0,1) << std::endl;
  cloud_model=cloud_target;
  cloud_model->header.frame_id="/object_model";
  pub3.publish(cloud_model);

  // tf::StampedTransform transform_center;
  // tf::Vector3 object_center;
  // object_center.setValue(tf::Vector3(center[0], center[1], center[2]);
  // transform.setOrigin(object_center);
  // transform_center.setRotation( tf::Quaternion(0, 0, 0, 1) );
  // tf_pub->sendTransform(tf::StampedTransform(transform_center, ros::Time::now(), "object_model", "/object_model_center"));
  // // tf::Transform transform_center;
  // // try{
  // //     tf_listener->waitForTransform("/table_top", "/object_model_center", ros::Duration(5.0));
  // //     tf_listener->lookupTransform ("/table_top", "/object_model_center", transform_center);
  // // }
  // // catch(std::runtime_error &e){
  // //     return;
  // // }

  tf::StampedTransform transform_final;

  cylinder.header.frame_id = "/object_model";
  cylinder.pose.orientation.w=1;
  cylinder.scale.x = shape[0];
  cylinder.scale.y = shape[1];
  cylinder.scale.z = shape[2];
  cylinder.pose.position.x = center[0];
  cylinder.pose.position.y = center[1];
  cylinder.pose.position.z = center[2];
  cylinder.action = visualization_msgs::Marker::ADD;
  marker_pub.publish(cylinder);
  ur5_ass::ObjectPose Object_Pose;
  geometry_msgs::PoseStamped object_pose;
  object_pose.header.frame_id="/object_model";
  object_pose.pose.position.x=center[0];
  object_pose.pose.position.y=center[1];
  object_pose.pose.position.z=center[2];
  object_pose.pose.orientation.w=1;
  Object_Pose.object_poses.push_back(object_pose);

  object_pose_pub.publish(Object_Pose);
  }
};

int main (int argc, char** argv)
{
  ros::init(argc, argv, "icp");
  ICPregister icp_register;
  ros::spin();
  return 0;
}
