#include "turtlebot_pid/trajectory_tracking_sim.h"

Trajectory_Tracking_Sim::Trajectory_Tracking_Sim(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("In class constructor of Trajectory_Tracking_Sim");
    initSub(); // package up the messy work of creating subscribers; do this overhead in constructor
    initPub();
    setvelocity();
    // setgoalpos();
    setpidgains();
    // setcircularpath();

}

void Trajectory_Tracking_Sim::setpidgains(double p, double i, double d)
{
    k_p = p;
    k_i = i;
    k_d = d;
}

void Trajectory_Tracking_Sim::setvelocity(double x) 
{
    v = x;
}

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
    currentpos_sub_ = nh_.subscribe("/odom", 1, &Trajectory_Tracking_Sim::currentposCallback,this);
    goalpos_sub_ = nh_.subscribe("/turtlebot/path", 1, &Trajectory_Tracking_Sim::goalposCallback,this); 
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

    // distance between goal and robot in x-direction
    u_x = goalpos.x - currentpos.x;
    // distance between goal and robot in y-direction
    u_y = goalpos.y - currentpos.y;
    // angle from robot to goal
    double theta_g = atan2(u_y, u_x);
    // error between the goal angle and robot's angle
    u_angle = theta_g - theta;
    e_k = atan2(sin(u_angle),cos(u_angle));

    // error for the proportional term
    e_P = e_k;
    // error for the integral term. Approximate the integral using the accumulated error, obj.E_k, and the error for this time step
    e_I = e_k + E_k;

    // cout << e_I << endl;
    // error for the derivative term. Hint: Approximate the derivative using the previous error, and the error for this time step, e_k.
    e_D = e_k - e_k_previous; 
    // update errors
    E_k = e_I;
    e_k_previous = e_k;

    // control input 
    w = k_p*e_P + k_i*e_I + k_d*e_D;

    if (w > 1.5)
    {
        w = 1.5;
    }
    else if (w<-1.5)
    {
        w = -1.5;
    }
    cout << w << endl;

    controlinput.angular.z = w;
    controlinput.linear.x = v;
    controlinput_pub_.publish(controlinput); //output the square of the received value; 
    ros::Duration(0.1).sleep();
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

