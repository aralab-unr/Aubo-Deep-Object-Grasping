/* Author: Ioan Sucan, Ridhwan Luthra*/

// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <gpd_ros/grasp_detection_node.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/*
 
void cloud_cb(const visualization_msgs::MarkerArrayConstPtr& grasp_pose_msg)
{
  std::cout << "This is the grasp pose:::::::::::::::::::::::::::::::::::::::::::            ";
  std::cout << grasp_pose_msg->pose.position.x << std::endl;
}
*/

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL open_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "gripper_finger1_joint";
  posture.joint_names[1] = "gripper_finger2_joint";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL closed_gripper
  /* Add both finger joints of aubo robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "gripper_finger1_joint";
  posture.joint_names[1] = "gripper_finger2_joint";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
  // BEGIN_SUB_TUTORIAL pick1
  // Create a vector of grasps to be attempted, currently only creating single grasp.
  // This is essentially useful when using a grasp generator to generate and test multiple grasps.
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // Setting grasp pose
  // ++++++++++++++++++++++
  
  grasps[0].grasp_pose.header.frame_id = "base_link";
  //tf2::Quaternion orientation;
  tf2::Quaternion orientation;
  orientation.setRPY(1.489, -0.059, -0.637);
  
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = -0.750;
  grasps[0].grasp_pose.pose.position.y = 0.041;
  grasps[0].grasp_pose.pose.position.z =  0.062;

 
  // Setting pre-grasp approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
  /* Direction is set as positive x axis */
  grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
  grasps[0].pre_grasp_approach.direction.vector.y = -1.0;
  grasps[0].pre_grasp_approach.direction.vector.x = -1.0;

  //grasps[0].pre_grasp_approach.min_distance = 0.20;
  grasps[0].pre_grasp_approach.desired_distance = 0.10;



  // Set support surface as table1.
  move_group.setSupportSurfaceName("table1");
  // Call pick to pick up the object using the grasps given
  move_group.pick("object", grasps);
  // END_SUB_TUTORIAL
}

void place(moveit::planning_interface::MoveGroupInterface& group)
{
  // 
  //  - Calling place function m rrently only creating single place location.
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // Setting place location pose
  // +++++++++++++++++++++++++++
  place_location[0].place_pose.header.frame_id = "world";
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, M_PI / 2);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  /* While placing it is the exact location of the center of the object. */
  place_location[0].place_pose.pose.position.x = 0;
  place_location[0].place_pose.pose.position.y = 0.5;
  place_location[0].place_pose.pose.position.z = 0.5;

  // Setting pre-place approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].pre_place_approach.direction.header.frame_id = "world";
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.z = -1.0;

  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].post_place_retreat.direction.header.frame_id = "world";
  /* Direction is set as negative y axis */
  place_location[0].post_place_retreat.direction.vector.x = -1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  // Setting posture of eef after placing object
  // +++++++++++++++++++++++++++++++++++++++++++
  /* Similar to the pick case */
  openGripper(place_location[0].post_place_posture);

  // Set support surface as table2.
  group.setSupportSurfaceName("table2");
  // Call place to place the object using the place locations given.
  group.place("object", place_location);
  // END_SUB_TUTORIAL
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // BEGIN_SUB_TUTORIAL table1
  //
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(1);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "base_link";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 1.0;
  collision_objects[0].primitives[0].dimensions[1] = 1.0;
  collision_objects[0].primitives[0].dimensions[2] = 0.47;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = -1.0;
  collision_objects[0].primitive_poses[0].position.y = 0.0;
  collision_objects[0].primitive_poses[0].position.z = -0.30;
  collision_objects[0].primitive_poses[0].orientation.x = 5;
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;

  

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aubo_arm_pick_place");
  ros::NodeHandle nh;
  //ros::NodeHandle m_nh;

//  ros::Subscriber sub = m_nh.subscribe("/detect_grasps/plot_grasps", 1, cloud_cb);
  //ros::spin();
  //ros::WallDuration(10.0).sleep();

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("aubo_arm");
  group.setPlanningTime(60.0);

  //addCollisionObjects(planning_scene_interface);

  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();

  pick(group);

  ros::WallDuration(1.0).sleep();

  //place(group);

  ros::waitForShutdown();
  return 0;
}

