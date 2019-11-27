cmake_minimum_required(VERSION 2.8.3)
project(ros_pub_and_sub_rgbd_and_cloud)

# compiler
set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )



############### ROS ######################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  
  std_msgs
  sensor_msgs
  geometry_msgs

)
# add_message_files( # add my message
#   FILES
#   T4x4.msg
# )
# add_service_files(
#   FILES
#   PrintBaxterGripperPose.srv
#   PrintBaxterJointAngles.srv
# )
# generate_messages( # dependencies to my message
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
#   baxter_core_msgs
# )

catkin_package(CATKIN_DEPENDS message_runtime) # declares dependencies for packages that depend on this package.

include_directories(
  ${catkin_INCLUDE_DIRS}
)
