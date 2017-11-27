#include<ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <ur5_ass/ObjectPose.h>
#include <geometry_msgs/PoseStamped.h>

#include<pcl_ros/point_cloud.h>
#include<pcl_ros/transforms.h>
#include<pcl_conversions/pcl_conversions.h>

#include<pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointXYZRGB Point;

class PointCould2Pose{
    private:
        ros::NodeHandle nh;
        ros::Subscriber sub1;
        ros::Subscriber sub2;
        ros::Subscriber sub3;
        ros::Subscriber sub4;
        ros::Subscriber sub5;
        tf::TransformListener *tf_listener;
        tf::TransformBroadcaster *tf_pub;
        ros::Publisher object_pose_pub;
        ros::Publisher target_pose_pub;
        ur5_ass::ObjectPose Object_Pose;

    public:
        PointCould2Pose(){
            sub1 = nh.subscribe<PointCloud>("filter point clould topic", 1, &ObjectRecognition::Pose_Generation, this);
            sub2 = nh.subscribe<PointCloud>("filter point clould topic", 1, &ObjectRecognition::Pose_Generation, this);
            sub3 = nh.subscribe<PointCloud>("filter point clould topic", 1, &ObjectRecognition::Pose_Generation, this);
            sub4 = nh.subscribe<PointCloud>("filter point clould topic", 1, &ObjectRecognition::Pose_Generation, this);
            sub5 = nh.subscribe<PointCloud>("filter point clould topic", 1, &ObjectRecognition::Pose_Generation, this);
            object_pose_pub = nh.advertise<ur5_ass::ObjectPose> ("Tower/ObjectPose", 1);
            tf_pub = new tf::TransformBroadcaster;
        }

        void Pose_Generation(const PointCloud::ConstPtr& msg_in){
            len=Object_Pose.object_id.size();
            int l=0;
            if (len==0)
               //for (int i=0; i<cloud_in.class.size(); i++) //every msg only have one class
                 {Object_Pose.object_id.push_back(msg.class);
                   //icp this pointcloud,then push_back this pose
                   //call some function in here
                   //geometry_msgs::PoseStamped pose(msg)
                  Object_Pose.object_poses.push_back(pose);
                }
            else
               {for (int j=0; j<len; j++)
                  if (cloud_in.class==bbb.object_id[j])
                     l=1;
               if (l==0)
                    {Object_Pose.object_id.push_back(msg.class);
                      //icp this pointcloud,then push_back this pose
                      //call some function in here
                      //geometry_msgs::PoseStamped pose(msg) //still nedd msg.class to identify
                     Object_Pose.object_poses.push_back(pose);
                   }
             }
             //if (len==5)
             object_pose_pub.publish(Object_Pose);

           }

        geometry_msgs::PoseStamped ICP_Kinect2(const PointCloud::ConstPtr& cloud_in){
          PointCloud::Ptr cloud_tf (new PointCloud);
          PointCloud::Ptr cloud_filtered (new PointCloud);
          //for (i=1,i<n,i++){}
          std_msgs::Header header = pcl_conversions::fromPCL(cloud_in->header);
          ROS_INFO_STREAM("cloudin_header is " <<header );

          tf::StampedTransform transform;
          try{
              tf_listener->waitForTransform("/table_top2", header.frame_id, header.stamp, ros::Duration(50.0));
              tf_listener->lookupTransform ("/table_top2", header.frame_id, header.stamp, transform);
              ROS_INFO("transforming");
          }
          catch(std::runtime_error &e){
              return;
              ROS_INFO("something is error when transform");
          }

          pcl_ros::transformPointCloud (*cloud_in, *cloud_tf, transform);
          cloud_tf->header.frame_id = "/table_top2";
          ROS_INFO("transform cloud_in frame into /table_top2");

          PointCloud::Ptr cloud_model(new PointCloud);
          pcl::PCDReader reader;
          reader.read ("/homeL/demo/ls_ws/src/ur5_ass/src/pcd/cylinder4.pcd", *cloud_model);
          std::cout << "PointCloud before filtering has: " << cloud_model->points.size () << " data points." << std::endl;
          cloud_model->header.frame_id="table_top2";
          Eigen::Vector4f min_pt;
          Eigen::Vector4f max_pt;
          pcl::getMinMax3D (*cloud_model, min_pt, max_pt);
          Eigen::Vector4f center = (max_pt - min_pt)/2 + min_pt;
          Eigen::Vector4f shape=max_pt - min_pt;

          Tm=translation(cloud_model,cloud_tf);
          tf::Vector3 origin;
          origin.setValue(static_cast<double>(Tm(0,3)),static_cast<double>(Tm(1,3)),static_cast<double>(Tm(2,3)));
          geometry_msgs::Quaternion geoqt;
          geoqt=quarternion_newframe(Tm，origin);

          object_pose.header.frame_id="/Robject+msg.class_name";
          object_pose.pose.position.x=center[0];
          object_pose.pose.position.y=center[1];
          object_pose.pose.position.z=center[2];
          object_pose.pose.orientation=geoqt;
          return object_pose;
        }

        Eigen::Matrix4f translation(const PointCloud::Ptr& cloud_model, PointCloud::Ptr& cloud_tf){
           pcl::IterativeClosestPoint<Point , Point> icp;
           icp.setInputCloud(cloud_model);
           icp.setInputTarget(cloud_tf);
           PointCloud Final;
           icp.align(Final);
           std::cout << "has converged:" << icp.hasConverged() << " score: " <<
           icp.getFitnessScore() << std::endl;
           std::cout << icp.getFinalTransformation() << std::endl;
           Eigen::Matrix4f Tm = icp.getFinalTransformation ();

           tf::Vector3 origin;
           origin.setValue(static_cast<double>(Tm(0,3)),static_cast<double>(Tm(1,3)),static_cast<double>(Tm(2,3)));
           return Tm;
        }

        geometry_msgs::Quaternion quarternion_newframe(Eigen::Matrix4f& Tm,tf::Vector3& origin){
          tf::Matrix3x3 tf3d;
          tf3d.setValue(static_cast<double>(Tm(0,0)), static_cast<double>(Tm(0,1)), static_cast<double>(Tm(0,2)),
          static_cast<double>(Tm(1,0)), static_cast<double>(Tm(1,1)),static_cast<double>(Tm(1,2)),
          static_cast<double>(Tm(2,0)), static_cast<double>(Tm(2,1)),static_cast<double>(Tm(2,2)));

          tf::Quaternion tfqt;
          geometry_msgs::Quaternion geoqt;
          tf3d.getRotation(tfqt);
          tf::quaternionTFToMsg(tfqt,geoqt);
          transform.setOrigin(origin);
          transform.setRotation(tfqt);
          tf_pub->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "table_top2", "/Robject+msg.class_name"));
          return geoqt;
        }

        ~PointCould2Pose(){}
};

int main(int argc, char** argv){
    ros::init(argc, argv, "pointcould2pose");
    PointCould2Pose pointcould2pose;
    ros::spin();
    return 0;
}
