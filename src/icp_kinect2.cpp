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

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>


typedef pcl::PointCloud<pcl::PointXYZ> point_cloud;
typedef pcl::PointXYZ Point;
class ICPregister{
private:
  ros::NodeHandle nh;
  ros::Publisher marker_pub;
  ros::Subscriber sub;
  tf::TransformBroadcaster *tf_pub;
  tf::TransformListener *tf_listener;
  ros::Publisher pub1;
  ros::Publisher pub2;

public:
  ICPregister(){
  sub = nh.subscribe<point_cloud>("kinect2/qhd/points", 1, &ICPregister::pointCloudCb,this);
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  tf_pub=new tf::TransformBroadcaster;
  tf_listener = new tf::TransformListener;
  pub1= nh.advertise<point_cloud> ("pointcloud1", 1);
  pub2= nh.advertise<point_cloud> ("pointcloud2", 1);
}
  void pointCloudCb(const point_cloud::ConstPtr& cloud_in){
  point_cloud::Ptr cloud_tf (new point_cloud);
  point_cloud::Ptr cloud_filtered (new point_cloud);
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

  point_cloud::Ptr cloud_target(new point_cloud);
  pcl::PCDReader reader;
  reader.read ("/homeL/demo/ws_ur/src/ur5_ass/src/cylinder4.pcd", *cloud_target);
  std::cout << "PointCloud before filtering has: " << cloud_target->points.size () << " data points." << std::endl;
  cloud_target->header.frame_id="table_top";
  Eigen::Vector4f min_pt;
  Eigen::Vector4f max_pt;
  pcl::getMinMax3D (*cloud_target, min_pt, max_pt);
  Eigen::Vector4f center = (max_pt - min_pt)/2 + min_pt;
  pub1.publish(cloud_target);

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputCloud(objectCloud);
  icp.setInputTarget(cloud_target);
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
  transform.setOrigin(origin);
  transform.setRotation(tfqt);
  tf_pub->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "table_top", "/object_model"));
  //ROS_INFO_STREAM("tfqt.x"<<transform.getRotation ());
  //std::cout << tfqt(0,1) << std::endl;

  cylinder.pose.orientation=geoqt;
  cylinder.scale.x = 0.049;
  cylinder.scale.y = 0.049;
  cylinder.scale.z = Tm(2,3);
  cylinder.pose.position.x = Tm(0,3)+center[0];
  cylinder.pose.position.y = Tm(1,3)+center[1];
  cylinder.pose.position.z = Tm(2,3)/2+center[2];
  cylinder.action = visualization_msgs::Marker::ADD;
  marker_pub.publish(cylinder);
  }
};

int main (int argc, char** argv)
{
  ros::init(argc, argv, "icp");
  ICPregister icp_register;
  ros::spin();
  return 0;
}
