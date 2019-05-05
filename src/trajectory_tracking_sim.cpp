#include "turtlebot_pid/trajectory_tracking_sim.h"

Trajectory_Tracking_Sim::Trajectory_Tracking_Sim(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("In class constructor of Trajectory_Tracking_Sim");
    initSub(); // package up the messy work of creating subscribers; do this overhead in constructor
    initPub();  
    setpidgains();
}

void Trajectory_Tracking_Sim::setpidgains(double vp, double vi, double vd, double wp, double wi, double wd)
{
    vk_p = vp;
    vk_i = vi;
    vk_d = vd;
    wk_p = wp;
    wk_i = wi;
    wk_d = wd;
}

// void Trajectory_Tracking_Sim::setvelocity(double x) 
// {
//     v = x;
// }

double Trajectory_Tracking_Sim::quatoeuler_yaw(const nav_msgs::Odometry& odom)
{
    tf::Quaternion qua(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    tf::Matrix3x3 mat(qua);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    return yaw;
}


void Trajectory_Tracking_Sim::initSub()
{
    ROS_INFO("Initializing Subscribers");  
    currentpos_sub_ = nh_.subscribe("/odom", 1000, &Trajectory_Tracking_Sim::currentposCallback,this);
    goalpos_sub_ = nh_.subscribe("/turtlebot/path", 1000, &Trajectory_Tracking_Sim::goalposCallback,this); 
}


//member helper function to set up publishers;
void Trajectory_Tracking_Sim::initPub()
{
    ROS_INFO("Initializing Publishers");
    controlinput_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1, true);
}

void Trajectory_Tracking_Sim::goalposCallback(const geometry_msgs::Pose2D& pos) 
{
    goalpos.x = pos.x;
    goalpos.y = pos.y;
} 

void Trajectory_Tracking_Sim::currentposCallback(const nav_msgs::Odometry& odom) 
{   

    currentpos.x = odom.pose.pose.position.x;
    currentpos.y = odom.pose.pose.position.y;

    double theta = quatoeuler_yaw(odom);

    ROS_INFO("goal X, goal Y=%1.2f  %1.2f", goalpos.x, goalpos.y);
    ROS_INFO("current X, current Y=%1.2f  %1.2f", currentpos.x, currentpos.y);
    // distance between goal and robot in x-direction
    u_x = goalpos.x - currentpos.x;
    // distance between goal and robot in y-direction
    u_y = goalpos.y - currentpos.y;
    // angle from robot to goal
    double theta_g = atan2(u_y, u_x);
    // error between the goal angle and robot's angle
    u_angle = theta_g - theta;

    // ve_k = u_x;
    ve_k = sqrt(pow(u_x, 2) + pow(u_y, 2));
    we_k = atan2(sin(u_angle),cos(u_angle));

    // error for the proportional term
    ve_P = ve_k;
    we_P = we_k;

    // error for the integral term. Approximate the integral using the accumulated error, obj.E_k, and the error for this time step
    ve_I = ve_k + vE_k;
    we_I = we_k + wE_k;

    // cout << e_I << endl;
    // error for the derivative term. Hint: Approximate the derivative using the previous error, and the error for this time step, e_k.
    ve_D = ve_k - ve_k_previous; 
    we_D = we_k - we_k_previous; 

    // update errors
    vE_k = ve_I;
    ve_k_previous = ve_k;
    wE_k = we_I;
    we_k_previous = we_k;

    // control input 
    v = vk_p*ve_P + vk_i*ve_I + vk_d*ve_D;
    w = wk_p*we_P + wk_i*we_I + wk_d*we_D;

    if (v > 0.4)
    {
        v = 0.4;
    }
    else if ( v < -0.4)
    {
        v = -0.4;
    }

    if (w > 1.5)
    {
        w = 1.5;
    }
    else if ( w < -1.5)
    {
        w = -1.5;
    }
    // cout << "##########" << v << endl;
    controlinput.angular.z = w;
    controlinput.linear.x = v;
    ROS_INFO("W input=, V input=%1.2f %1.2f", w, v);
    controlinput_pub_.publish(controlinput); //output the square of the received value; 
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "turtlebot_gotogoal_sim"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    Trajectory_Tracking_Sim trajectory_tracking_sim(&nh);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("Initializing...");
    ros::spin();
    return 0;
} 

