#ifndef TRAJECTORY_TRACKING_SIM_H_
#define TRAJECTORY_TRACKING_SIM_H_

#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include "tf/transform_datatypes.h"

// https://answers.ros.org/question/31779/how-to-use-a-message-that-i-made/

using namespace std;

class Trajectory_Tracking_Sim
{
public:
    Trajectory_Tracking_Sim(ros::NodeHandle* nodehandle)   ;

private:
	// const double v_max = 2.0;
    // const double w_max = 2.0;


    // control input for linear velocity
    double v = 0;
    double w = 0;

    // pid gain parameters
    double vk_p;
    double vk_d;
    double vk_i;
    double wk_p;
    double wk_d;
    double wk_i;

    // error dynamics
    double ve_P = 0;
    double ve_I = 0;
    double ve_D= 0;
    double we_P = 0;
    double we_I = 0;
    double we_D= 0;

    double u_x = 0;
    double u_y = 0;

    double u_angle = 0;

    // accumulated error
    double vE_k = 0;
    double ve_k;
    double wE_k = 0;
    double we_k;
    // previous error
    double ve_k_previous = 0;
    double we_k_previous = 0;

    geometry_msgs::Pose2D currentpos, goalpos;
    geometry_msgs::Twist controlinput;

    ros::NodeHandle nh_;
    ros::Subscriber goalpos_sub_;
    ros::Subscriber currentpos_sub_;
    ros::Publisher controlinput_pub_;

    void initSub(); 
    void initPub();


    void setpidgains(double vp=0.10, double vi=0.001, double vd=0.05, double wp=5.0, double wi=0.01, double wd=0.05);
    double quatoeuler_yaw(const nav_msgs::Odometry& odom);

    void goalposCallback(const geometry_msgs::Pose2D& pos); 
    void currentposCallback(const nav_msgs::Odometry& odom); 

};

#endif

    // https://github.com/wsnewman/ros_class/blob/master/example_ros_class/src/example_ros_class.h