#include<ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <ur5_ass/ObjectPose.h>
#include <ur5_ass/TargetPose.h>
#include <geometry_msgs/PoseStampe.h>

#include<pcl_ros/point_cloud.h>
#include<pcl_ros/transforms.h>
#include<pcl_conversions/pcl_conversions.h>

#include<pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

class PointCould2Pose{
    private:
        ros::NodeHandle nh;
        ros::Subscriber sub;
        tf::TransformListener *tf_listener;
        tf::TransformBroadcaster *tf_pub;
        ros::Publisher object_pose_pub;
        ros::Publisher target_pose_pub;
        ur5_ass::TargetPose Target_Pose

    public:
        PointCould2Pose(){
            sub = nh.subscribe<PointCloud>("filter point clould topic", 1, &ObjectRecognition::pointCloudCb, this);
            target_pose_pub = nh.advertise<ur5_ass::TargetPose> ("Tower/TargetPose", 1);
            tf_listener = new tf::TransformListener;
            tf_pub = new tf::TransformBroadcaster;
        }

        void pointCloudCb(const PointCloud::ConstPtr& cloud_in){
            PointCloud::Ptr cloud_tf (new PointCloud);
            PointCloud::Ptr cloud_filtered (new PointCloud);
            //for (i=1,i<n,i++){}
            std_msgs::Header header = pcl_conversions::fromPCL(cloud_in->header);
            ROS_INFO_STREAM("cloudin_header is " <<header );

            tf::StampedTransform transformH;
            try{
                tf_listener->waitForTransform("/table_top", header.frame_id, header.stamp, ros::Duration(50.0));
                tf_listener->lookupTransform ("/table_top", header.frame_id, header.stamp, transform);
                ROS_INFO("transforming");
            }
            catch(std::runtime_error &e){
                return;
                ROS_INFO("something is error when transform");
            }

            pcl_ros::transformPointCloud (*cloud_in, *cloud_tf, transform);
            cloud_tf->header.frame_id = "/table_top1";
            ROS_INFO("transform cloud_in frame into /table_top1");

            pcl::PCLPointCloud2::Ptr cloudmodel (new pcl::PCLPointCloud2);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_model (new pcl::PointCloud<pcl::PointXYZ>)
            //if class_name==""
                pcl::io::loadPCDFile("cylinder4.pcd", *cloudmodel);
                Object_Pose.object_id.push_back("class_name");

            pcl::fromPCLPointCloud2 (*cloudmodel, *cloud_model);

            Tm=translation(cloud_model,cloud_tf);
            tf::Vector3 origin;
            origin.setValue(static_cast<double>(Tm(0,3)),static_cast<double>(Tm(1,3)),static_cast<double>(Tm(2,3)));
            geoqt=quarternion_newframe(Tm，origin);

            geometry_msgs::PoseStamped target_pose;
            target_pose.header.frame_id="/table_top2";
            target_pose.pose.position.x=Tm(0,3);
            target_pose.pose.position.y=Tm(1,3);
            target_pose.pose.position.z=Tm(2,3);
            target_pose.pose.orientation=geoqt;
            Target_Pose.target_pose=target_pose;

            object_pose_pub.publish(Object_Pose);
        }

        Eigen::Matrix4f translation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_model, PointCloud::Ptr& cloud_tf){
           pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
           icp.setInputCloud(cloud_model);
           icp.setInputTarget(cloud_tf);
           pcl::PointCloud<pcl::PointXYZ> Final;
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
          transformH.setOrigin(origin);
          transformH.setRotation(tfqt);
          tf_pub->sendTransform(tf::StampedTransform(transformH, ros::Time::now(), "table_top2", "/Rplace"+class_name));
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
