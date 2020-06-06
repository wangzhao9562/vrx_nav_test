/*
 * @Author: Zhao Wang
 * @Date: 2020-06-06 10:44:10
 * @LastEditTime: 2020-06-06 11:08:11
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /vrx_nav_test/test/kinematic_odom_node.cpp
 */ 

#include "vrx_nav_test/kinematic_odom.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kinematic_odom_node");
    
    vrx_nav_test::KinematicOdom(10.0, 50.0, 0.0, 0.0);

    ros::spin();

    return 0;
}
