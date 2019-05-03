#ifndef TRAJECTORY_TRACKING_SIM_H_
#define TRAJECTORY_TRACKING_SIM_H_

#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
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
    double v;
    double w = 0;

    // pid gain parameters
    double k_p;
    double k_d;
    double k_i;

    // error dynamics
    double e_P = 0;
    double e_I = 0;
    double e_D= 0;

    double u_x = 0;
    double u_y = 0;

    double u_angle = 0;

    // accumulated error
    double E_k = 0;
    double e_k;
    // previous error
    double e_k_previous = 0;

    geometry_msgs::Pose2D currentpos, goalpos;
    geometry_msgs::Twist controlinput;

    ros::NodeHandle nh_;
    ros::Subscriber goalpos_sub_;
    ros::Subscriber currentpos_sub_;
    ros::Publisher controlinput_pub_;


    void initSub(); 
    void initPub();

    double heading_angle(double x, double y);
    void setpidgains(double p=3.0, double i=0, double d=0);
    void setvelocity(double x=0.5);
    double quatoeuler_yaw(const nav_msgs::Odometry& odom);

    void goalposCallback(const geometry_msgs::Pose2D& pos); 
    void currentposCallback(const nav_msgs::Odometry& odom); 

};

#endif

    // https://github.com/wsnewman/ros_class/blob/master/example_ros_class/src/example_ros_class.h