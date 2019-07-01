#ifndef GO_TO_ANGLE_SIM_H_
#define GO_TO_ANGLE_SIM_H_

#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include "tf/transform_datatypes.h"
// #include "LinearMath/btMatrix3x3.h"

// https://answers.ros.org/question/31779/how-to-use-a-message-that-i-made/

using namespace std;

class GotoAngle_Sim
{
public:
    GotoAngle_Sim(ros::NodeHandle* nodehandle);

private:
	// const double v_max = 2.0;
    // const double w_max = 2.0;

    // control input for linear velocity
    double v;
    double w;
    double theta_d;

    // pid gain parameters
    double k_p;
    double k_d;
    double k_i;

    // error dynamics
    double e_P;
    double e_I;
    double e_D;

    double u_x;
    double u_y;

    double u_angle;

    // accumulated error
    double E_k;
    double e_k;
    // previous error
    double e_k_previous;

    geometry_msgs::Pose2D currentpos, goalpos;
    geometry_msgs::Twist controlinput;

    ros::NodeHandle nh_;
    ros::Subscriber stop_sub_;
    ros::Subscriber currentpos_sub_;
    ros::Publisher controlinput_pub_;

    void initSub(); 
    void initPub();
    void setgoalpos(double x=-1);
    void setpidgains(double p=3.0, double i=0, double d=1.0);
    void setvelocity(double x=0.5);
    double quatoeuler_yaw(const nav_msgs::Odometry& odom);

    void currentposCallback(const nav_msgs::Odometry& odom); 
    void stopCallback(const nav_msgs::Odometry& odom); 

};

#endif

    // https://github.com/wsnewman/ros_class/blob/master/example_ros_class/src/example_ros_class.h