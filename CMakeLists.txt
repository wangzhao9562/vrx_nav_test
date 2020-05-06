cmake_minimum_required(VERSION 2.8.3)
project(vrx_nav_test)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  map_server
  move_base
  hector_mapping
  hector_trajectory_server
  hector_geotiff
  nodelet
  robot_state_publisher
  roscpp
  rospy
  vrx_gazebo
  wamv_description
  wamv_gazebo
)

# find_package(Boost REQUIRED COMPONENTS system)


# catkin_python_setup()


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

