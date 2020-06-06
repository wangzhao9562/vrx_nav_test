/*
 * @Author: Zhao Wang
 * @Date: 2020-06-05 21:33:14
 * @LastEditTime: 2020-06-06 12:45:36
 * @LastEditors: Please set LastEditors
 * @Description: Odometry calculated through kinematic model of robot
 * @FilePath: /vrx_nav_test/include/vrx_nav_test/kinematic_odom.h
 */ 

#ifndef KINEMATIC_ODOM_H_
#define KINEMATIC_ODOM_H_

#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<nav_msgs/Odometry.h>
#include<boost/thread.hpp>
#include<string>
#include<pthread.h>

namespace vrx_nav_test{
class KinematicOdom{
public:
    KinematicOdom(double init_x, double init_y, double init_z, double init_th);
    ~KinematicOdom();

private:
    void velocityUpdate(const geometry_msgs::Twist::ConstPtr& vel);

    void updateOdom();

private:
    int publish_freq_; // publish frequency
    std::string global_frame_; // frame of map
    std::string odom_frame_; // frame of odom
    std::string base_frame_;

    ros::Time current_time_;
    ros::Time last_time_;

    nav_msgs::Odometry odom_;
    nav_msgs::Odometry global_odom_;
    boost::thread* odom_pub_t_;

    geometry_msgs::Twist vel_;

    pthread_rwlock_t flock_;

    ros::Subscriber vel_sub_; // subscriber of velocity commands
    ros::Publisher global_odom_pub_; 
    ros::Publisher odom_pub_; // publisher of robot odometry

}; // end of class

} // end of namespace

#endif