#ifndef GO_TO_GOAL_VW_Sim_H_
#define GO_TO_GOAL_VW_Sim_H_

#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>

// https://answers.ros.org/question/31779/how-to-use-a-message-that-i-made/

using namespace std;

class GotoGoal_VW_Sim
{
public:
    GotoGoal_VW_Sim(ros::NodeHandle* nodehandle)   ;

private:
	// const double v_max = 2.0;
    // const double w_max = 2.0;


    // control input for linear velocity
    double v = 0;
    double w = 0;
    double vx = 0;
    double vy = 0;

    // pid gain parameters
    double kp_x, kd_x, ki_x;
    double kp_y, kd_y, ki_y;
    double kp_theta, kd_theta, ki_theta;

    // error dynamics
    double eP_x = 0;
    double eI_x = 0;
    double eD_x= 0;

    double eP_y = 0;
    double eI_y = 0;
    double eD_y= 0;

    double eP_theta = 0;
    double eI_theta = 0;
    double eD_theta= 0;

    double u_x = 0;
    double u_y = 0;
    double u_theta = 0;

    // accumulated error
    double Ek_x = 0;
    double Ek_y = 0;
    double Ek_theta = 0;

    double ek_x, ek_y, ek_theta;

    // previous error
    double ek_x_previous = 0;
    double ek_y_previous = 0;
    double ek_theta_previous = 0;

    geometry_msgs::Pose2D currentpos, goalpos;
    geometry_msgs::Twist controlinput;

    ros::NodeHandle nh_;
    // ros::Subscriber goalpos_sub_;
    ros::Subscriber currentpos_sub_;
    ros::Subscriber stop_sub_;
    ros::Publisher controlinput_pub_;

    void initSub(); 
    void initPub();

    void setgoalpos(double x=5.0, double y=-2.0, double theta=0.2);
    void setpidgains(double px=5.0, double ix=0, double dx=1.0, double py=8.0, double iy=0, double dy=1.0, double ptheta=10.0, double itheta=0, double dtheta=1.0);

    // void goalposCallback(const vicon_bridge::Marker& markers); 
    // void currentposCallback(const vicon_bridge::Marker& markers); 
    void currentposCallback(const nav_msgs::Odometry& odom); 
    void stopCallback(const nav_msgs::Odometry& odom); 

};

#endif

    // https://github.com/wsnewman/ros_class/blob/master/example_ros_class/src/example_ros_class.h