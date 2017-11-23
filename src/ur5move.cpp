/* Author: Shuang Li */
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
//#include <sensor_msgs/JointState.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>

int main(int argc, char **argv)
{
  ros::init(argc,argv,"ur5move");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  sleep(20);
  moveit::planning_interface::MoveGroupInterface group("arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  // move to "extended" pose

  group.setNamedTarget("pour_default_2");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = group.move();
  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(5.0);

  // setJointValueTarget
  //group.setStartStateToCurrentState ();
  geometry_msgs::PoseStamped target2;
  //target2.header.frame_id="table_top";??can't work when set the pose with respenct to "table_top"
  target2.header.frame_id="/world";
  target2.header.stamp=ros::Time::now();
  target2.pose.orientation.w=1;
  target2.pose.position.x=0.957917615558;// 0.371849018129
  target2.pose.position.y=0.566543526334;// 0.298981806364
  target2.pose.position.z= 1.93391489751;//1.80035830512
  group.setJointValueTarget(target2);
  //[ INFO] [1506606463.435696522]: Combined planning and execution request received for MoveGroup action. Forwarding to planning and execution pipeline.
//[ INFO] [1506606463.435857025]: Goal constraints are already satisfied. No need to plan or execute any motions
// if position.x=0.371849018129;
//[ INFO] [1506606615.729826846]: Collision checking is considered complete (collision was found and 0 contacts are stored)
//[ERROR] [1506606615.827131251]: arm[RRTConnectkConfigDefault]: Unable to sample any valid states for goal tree
//[ INFO] [1506606615.827246447]: arm[RRTConnectkConfigDefault]: Created 1 states (1 start + 0 goal)
//[ INFO] [1506606615.827291573]: No solution found after 0.202072 seconds
//[ INFO] [1506606615.827366022]: Unable to solve the planning problem
//[ INFO] [1506606615.828323762]: ABORTED: No motion plan found. No execution attempted.

  success = group.move();
  ROS_INFO("Visualizing plan 2 (pose goal) %s",success?"":"FAILED");
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(4.0);

  group.setStartStateToCurrentState ();
  group.setNamedTarget("extended");
  success = group.move();
  ROS_INFO("Visualizing plan 3 (pose goal move around box) %s",
    success?"":"FAILED");
  sleep(10.0);

  // Adding/Removing Objects and Attaching/Detaching Objects
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // First, we will define the collision object message.
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "table_top";
  collision_object.id = "box1";

  /* Define a box to add to the world. */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.1;
  primitive.dimensions[1] = 0.2;
  primitive.dimensions[2] = 0.4;

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x =  -0.1;
  box_pose.position.y =-0.2;
  box_pose.position.z = 0.6;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  // Now, let"s add the collision object into the world
  ROS_INFO("Add an object into the world");
  planning_scene_interface.applyCollisionObject(collision_object);

  /* Sleep so we have time to see the object in RViz */
  sleep(2.0);

  // Planning with collision detection can be slow.  Lets set the planning time
  // to be sure the planner has enough time to plan around the box.  10 seconds
  // should be plenty.
  group.setPlanningTime(10.0);

  // Now when we plan a trajectory it will avoid the obstacle
  //setJointValueTarget
  //robot_state::RobotState start_state(*group.getCurrentState());
  //group.setStartState(start_state);
  std::vector<double> group_joint_values;
  group_joint_values=group.getCurrentJointValues();
  //ROS_INFO_STREAM("getCurrentJointValues"<<group_joint_values[0]);
  group_joint_values[0]=0.2945793125912239;
  group_joint_values[1]=-1.320314771497747;
  group_joint_values[4]=0.2478303693304302;
  group.setJointValueTarget(group_joint_values);
  success= group.move();
  ROS_INFO("Visualizing plan 4 (pose goal) %s",success?"":"FAILED");
  sleep(5.0);

  // Now, let"s remove the collision object from the world.
  ROS_INFO("Remove the object from the world");
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  planning_scene_interface.removeCollisionObjects(object_ids);
  /* Sleep to give Rviz time to show the object is no longer there. */
  sleep(4.0);

  ros::shutdown();
  return 0;
}
