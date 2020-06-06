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
  nav_msgs
  geometry_msgs
)

find_package(Boost REQUIRED COMPONENTS system)


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

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(kinematic_odom
  src/kinematic_odom.cpp
)

target_link_libraries(kinematic_odom
  ${catkin_LIBRARIES}
)

add_dependencies(kinematic_odom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(kinematic_odom_node
  test/kinematic_odom_node.cpp
)

add_dependencies(kinematic_odom_node ${${PROJECT_NAME}catkin_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(kinematic_odom_node
  kinematic_odom
  ${catkin_LIBRARIES}
)

