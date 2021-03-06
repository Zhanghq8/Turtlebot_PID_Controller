#ifndef FOLLOW_WALL_SIM_H_
#define FOLLOW_WALL_SIM_H_

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

class Follow_Wall_Sim
{
public:
    Follow_Wall_Sim(ros::NodeHandle* nodehandle);

private:
	// const double v_max = 2.0;
    // const double w_max = 2.0;

    // control input for linear velocity
    double v_normal;  //velocity for avoiding obstacle
    double v_ao;
    double w;

    //follow the left or righr wall
    int direction; // left:1, right: -1

    // pid gain parameters
    double k_p;
    double k_d;
    double k_i;

    // error dynamics
    double e_P;
    double e_I;
    double e_D;

    // accumulated error
    double E_k;
    double e_k;
    // previous error
    double e_k_previous;

    //distance to the wall 
    double diswall;

    //obstacle pos
    double laserdis[5];
    double obstacle_pos[5][2];
    double lasergain[5] = {1, 1, 1, 1, 1};

    geometry_msgs::Pose2D currentpos, goalpos;
    geometry_msgs::Twist controlinput;

    ros::NodeHandle nh_;
    ros::Subscriber currentpos_sub_;
    ros::Subscriber laserpos_sub_;
    ros::Publisher controlinput_pub_;


    void initSub(); 
    void initPub();

    void setpidgains(double p=3.0, double i=0.0, double d=0.01);
    void setvelocity(double x=0.3, double y=0.05);
    void setdirection(int x=-1);//-1:clockwise; 1:counter-clockwise
    double quatoeuler_yaw(const nav_msgs::Odometry& odom);
    

    void currentposCallback(const nav_msgs::Odometry& odom);
    void laserCallback(const sensor_msgs::LaserScan& scan);
    void stopCallback(const nav_msgs::Odometry& odom); 


};

#endif

    // https://github.com/wsnewman/ros_class/blob/master/example_ros_class/src/example_ros_class.h
