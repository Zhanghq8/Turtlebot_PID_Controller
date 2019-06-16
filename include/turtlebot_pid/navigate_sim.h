#ifndef NAVIGATE_SIM_H_
#define NAVIGATE_SIM_H_

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

class Navigate_Sim
{
public:
    Navigate_Sim(ros::NodeHandle* nodehandle)   ;

private:
	// const double v_max = 2.0;
    // const double w_max = 2.0;

    // control input for linear velocity
    double v_normal;  //velocity for avoiding obstacle
    double v_ao;
    double w = 0;

    // //follow the left or righr wall
    // int direction; // left:1, right: -1

    // pid gain parameters
    double k_p;
    double k_d;
    double k_i;

    // error dynamics
    double e_P = 0;
    double e_I = 0;
    double e_D= 0;

    // accumulated error
    double E_k = 0;
    double e_k;
    // previous error
    double e_k_previous = 0;

    double count = 0.0;
    //distance to the wall 
    double diswall = 0.70;

    // safe distance
    double distance_safe = 0.60;

    //progress distance
    double distance_gtg_fw = 10000.0;

    //obstacle pos
    double laserdis[5];
    double obstacle_pos[5][2];
    double lasergain[5] = {1, 1, 1, 1, 1};

    string state_current = "init";
    string state_last = "init";

    geometry_msgs::Pose2D currentpos, goalpos;
    geometry_msgs::Twist controlinput;

    ros::NodeHandle nh_;
    ros::Subscriber currentpos_sub_;
    ros::Subscriber laserpos_sub_;
    ros::Subscriber stop_sub_;
    ros::Publisher controlinput_pub_;


    void initSub(); 
    void initPub();

    void setgoalpos(double x=-3.0, double y=-3.0);
    void setpidgains(double p=3.0, double i=0.0, double d=0.01);
    void setvelocity(double x=0.3, double y=0.05);
    double quatoeuler_yaw(const nav_msgs::Odometry& odom);
    

    void currentposCallback(const nav_msgs::Odometry& odom);
    void laserCallback(const sensor_msgs::LaserScan& scan);
    void stopCallback(const nav_msgs::Odometry& odom); 


};

#endif

    // https://github.com/wsnewman/ros_class/blob/master/example_ros_class/src/example_ros_class.h
