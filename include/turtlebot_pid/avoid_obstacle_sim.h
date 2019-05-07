#ifndef AVOID_OBSTACLE_SIM_H_
#define AVOID_OBSTACLE_SIM_H_

#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include "tf/transform_datatypes.h"
#include <sensor_msgs/LaserScan.h>

// https://answers.ros.org/question/31779/how-to-use-a-message-that-i-made/

using namespace std;

class Avoid_Obstacle_Sim
{
public:
    Avoid_Obstacle_Sim(ros::NodeHandle* nodehandle)   ;

private:
	// const double v_max = 2.0;
    // const double w_max = 2.0;


    // control input for linear velocity
    double v;
    double w = 0;
    // double theta_d = 0.5;

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

    //obstacle pos
    double obstacle_pos[5][2];
    double lasergain[5] = {1.3, 1, 1, 1, 1};

    geometry_msgs::Pose2D currentpos;
    geometry_msgs::Twist controlinput;

    ros::NodeHandle nh_;
    ros::Subscriber currentpos_sub_;
    ros::Subscriber laserpos_sub_;
    ros::Publisher controlinput_pub_;

    void initSub(); 
    void initPub();

    void setpidgains(double p=5.0, double i=0.0, double d=0.05);
    void setvelocity(double x=0.3);
    double quatoeuler_yaw(const nav_msgs::Odometry& odom);
    

    void currentposCallback(const nav_msgs::Odometry& odom);
    void laserCallback(const sensor_msgs::LaserScan& scan);


};

#endif

    // https://github.com/wsnewman/ros_class/blob/master/example_ros_class/src/example_ros_class.h
