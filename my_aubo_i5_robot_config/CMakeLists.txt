cmake_minimum_required(VERSION 2.8.3)
project(my_aubo_i5_robot_config)
add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
find_package(PCL 1.9 REQUIRED)


find_package(catkin REQUIRED COMPONENTS 
  interactive_markers
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  geometric_shapes
  pcl_ros
  gpd_ros
  pcl_conversions
  rosbag
  tf2_ros
  tf2_eigen
  darknet_ros_msgs
  tf2_geometry_msgs
  roscpp
  rospy
  std_msgs
  message_filters
  geometry_msgs
  message_generation
)

catkin_package(
  LIBRARIES
    interactivity_utils
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
)

include_directories(${catkin_INCLUDE_DIRS})
##catkin_package()
find_package(Boost REQUIRED system filesystem date_time thread)
add_executable(pick_place src/pick_place.cpp)
target_link_libraries(pick_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pass_through src/pass_through.cpp)
target_link_libraries (pass_through ${catkin_LIBRARIES} ${PCL_LIBRARIES})

include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

install(TARGETS pick_place DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

##add_subdirectory(/home/nicholasward2/catkin_ws/src/moveit_tutorials/doc/kinematics)
##add_subdirectory(/home/nicholasward2/catkin_ws/src/moveit_tutorials/doc/robot_model_and_robot_state)
##add_subdirectory(/home/nicholasward2/catkin_ws/src/moveit_tutorials/doc/planning)
##add_subdirectory(/home/nicholasward2/catkin_ws/src/moveit_tutorials/doc/planning_scene)
##add_subdirectory(/home/nicholasward2/catkin_ws/src/moveit_tutorials/doc/planning_scene_ros_api)
##add_subdirectory(/home/nicholasward2/catkin_ws/src/moveit_tutorials/doc/motion_planning_api)
##add_subdirectory(/home/nicholasward2/catkin_ws/src/moveit_tutorials/doc/motion_planning_pipeline)
##add_subdirectory(/home/nicholasward2/catkin_ws/src/moveit_tutorials/doc/visualizing_collisions)
##add_subdirectory(/home/nicholasward2/catkin_ws/src/moveit_tutorials/doc/move_group_interface)
##add_subdirectory(/home/nicholasward2/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface)
##add_subdirectory(/home/nicholasward2/catkin_ws/src/moveit_tutorials/doc/state_display)
##add_subdirectory(/home/nicholasward2/catkin_ws/src/moveit_tutorials/doc/interactivity)
##add_subdirectory(/home/nicholasward2/catkin_ws/src/moveit_tutorials/doc/pick_place)
##add_subdirectory(/home/nicholasward2/catkin_ws/src/moveit_tutorials/doc/perception_pipeline)


