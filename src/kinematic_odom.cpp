/*
 * @Author: your name
 * @Date: 2020-06-05 21:33:26
 * @LastEditTime: 2020-06-06 13:02:56
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /vrx_nav_test/src/kinematic_odom.cpp
 */ 
#include"vrx_nav_test/kinematic_odom.h"
#include<boost/bind.hpp>
#include<tf/transform_datatypes.h>

namespace vrx_nav_test{
    KinematicOdom::KinematicOdom(double init_x, double init_y, double init_z, double init_th){
        ROS_INFO("Initialize parameters");

        ros::NodeHandle nh;
        nh.param("publish_frequency", publish_freq_, 20);
        nh.param("global_frame", global_frame_, std::string("wamv/map"));
        nh.param("odom_frame", odom_frame_, std::string("wamv/odom"));
        nh.param("base_frame", base_frame_, std::string("wamv/base_link"));

        ROS_INFO("Initialize ros components");
        vel_sub_ = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, boost::bind(&KinematicOdom::velocityUpdate, this, _1));
        odom_pub_ = nh.advertise<nav_msgs::Odometry>("wamv/odometry_odom", 1);
        global_odom_pub_ = nh.advertise<nav_msgs::Odometry>("wamv/odometry_map", 1);

        ROS_INFO("Initialize odometry");
        /* Reserved for odometry initialization */
        global_odom_.header.frame_id = global_frame_;
        global_odom_.child_frame_id = base_frame_;
        global_odom_.header.stamp = ros::Time::now();
        global_odom_.pose.pose.position.x = init_x;
        global_odom_.pose.pose.position.y = init_y;
        global_odom_.pose.pose.position.z = init_z;
        global_odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(init_th);
        global_odom_.twist.twist.linear.x = 0.0;
        global_odom_.twist.twist.angular.x = 0.0;
        global_odom_.twist.twist.linear.y = 0.0;
        global_odom_.twist.twist.linear.z = 0.0;
        global_odom_.twist.twist.angular.y = 0.0;
        global_odom_.twist.twist.angular.z = 0.0;

        odom_.header.frame_id = odom_frame_;
        odom_.child_frame_id = base_frame_;
        odom_.header.stamp = ros::Time::now();
        odom_.pose.pose.position.x = 0.0;
        odom_.pose.pose.position.y = 0.0;
        odom_.pose.pose.position.z = 0.0;
        odom_.pose.pose.orientation.w = 1.0;
        odom_.twist.twist.linear.x = 0.0;
        odom_.twist.twist.linear.y = 0.0;
        odom_.twist.twist.linear.z = 0.0;
        odom_.twist.twist.angular.x = 0.0;
        odom_.twist.twist.angular.y = 0.0;
        odom_.twist.twist.angular.z = 0.0;

        vel_.linear.x = 0.0;
        vel_.linear.y = 0.0;
        vel_.linear.z = 0.0;
        vel_.angular.x = 0.0;
        vel_.angular.y = 0.0;
        vel_.angular.z = 0.0;

        current_time_ = ros::Time::now();
        last_time_ = ros::Time::now();

        flock_ = PTHREAD_RWLOCK_INITIALIZER;

        ROS_INFO("Start odometry update thread");
        odom_pub_t_ = new boost::thread(boost::bind(&KinematicOdom::updateOdom, this));
    }

    KinematicOdom::~KinematicOdom(){
        // Free sources
        if(odom_pub_t_)
        {
            delete odom_pub_t_;
            odom_pub_t_ = nullptr;
        }
    }

    void KinematicOdom::velocityUpdate(const geometry_msgs::Twist::ConstPtr& vel)
    {
        // Update velocity
        pthread_rwlock_wrlock(&flock_);
        vel_.linear.x = vel->linear.x;
        vel_.linear.y = vel->linear.y;
        vel_.angular.z = vel->angular.z;
        pthread_rwlock_unlock(&flock_);
    }

    void KinematicOdom::updateOdom()
    {
        ROS_INFO("Set publish frequency");
        ros::Rate rate(publish_freq_);

        while(ros::ok()){
            // Get delta time
            current_time_ = ros::Time::now();
            double dt = (current_time_ - last_time_).toSec();

            // Get posture
            double vel_x = vel_.linear.x;
            double vel_y = vel_.linear.y;
            double vel_th = vel_.angular.z;

            ROS_INFO("Update odom odometry");
            // Update odometry
            double th_odom = tf::getYaw(odom_.pose.pose.orientation);
            double delta_x_odom = (vel_.linear.x * std::cos(th_odom) - vel_.linear.y * std::sin(th_odom)) * dt;
            double delta_y_odom = (vel_.linear.y * std::sin(th_odom) - vel_.linear.y * std::cos(th_odom)) * dt;
            double delta_th = vel_th * dt;

            odom_.pose.pose.position.x += delta_x_odom;
            odom_.pose.pose.position.y += delta_y_odom;
            double cur_th = th_odom + delta_th;
            odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(cur_th);
            odom_.twist.twist.linear.x = vel_x;
            odom_.twist.twist.linear.y = vel_y;
            odom_.twist.twist.angular.z = vel_th;

            // Update global odometry
            ROS_INFO("Update global odometry");
            double th_global = tf::getYaw(global_odom_.pose.pose.orientation);
            double delta_x_global = (vel_.linear.x * std::cos(th_global) - vel_.linear.y * std::sin(th_global)) * dt;
            double delta_y_global = (vel_.linear.y * std::sin(th_global) - vel_.linear.y * std::cos(th_global)) * dt;

            global_odom_.pose.pose.position.x += delta_x_global;
            global_odom_.pose.pose.position.y += delta_y_global;
            double cur_th_global = th_global + delta_th;
            global_odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(cur_th_global);
            global_odom_.twist.twist.linear.x = vel_x;
            global_odom_.twist.twist.linear.y = vel_y;
            global_odom_.twist.twist.angular.z = vel_th;

            ROS_INFO("Publish odom odometry");
            odom_pub_.publish(odom_);
            ROS_INFO("Publish global odometry");
            global_odom_pub_.publish(global_odom_);

            last_time_ = current_time_;

            rate.sleep(); 
        }
    }

} // end of namespace