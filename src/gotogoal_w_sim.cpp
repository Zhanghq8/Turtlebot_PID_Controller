#include "turtlebot_pid/gotogoal_w_sim.h"

GotoGoal_W_Sim::GotoGoal_W_Sim(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("In class constructor of GotoGoal_W_Sim");
    initSub(); // package up the messy work of creating subscribers; do this overhead in constructor
    initPub();
    setvelocity();
    setgoalpos();
    setpidgains();

    w = 0;

    // error dynamics
    e_P = 0;
    e_I = 0;
    e_D= 0;

    u_x = 0;
    u_y = 0;

    u_angle = 0;

    // accumulated error
    E_k = 0;
    // previous error
    e_k_previous = 0;
}

void GotoGoal_W_Sim::setpidgains(double p, double i, double d)
{
    k_p = p;
    k_i = i;
    k_d = d;
}

void GotoGoal_W_Sim::setvelocity(double x) 
{
    v = x;
}

void GotoGoal_W_Sim::setgoalpos(double x, double y)
{
    goalpos.x = x;
    goalpos.y = y;
}

double GotoGoal_W_Sim::quatoeuler_yaw(const nav_msgs::Odometry& odom)
{
    tf::Quaternion qua(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    tf::Matrix3x3 mat(qua);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    return yaw;
}

void GotoGoal_W_Sim::initSub()
{
    ROS_INFO("Initializing Subscribers");  
    currentpos_sub_ = nh_.subscribe("/odom", 1, &GotoGoal_W_Sim::currentposCallback,this); 
    stop_sub_ = nh_.subscribe("/odom", 1, &GotoGoal_W_Sim::stopCallback,this);
}


//member helper function to set up publishers;
void GotoGoal_W_Sim::initPub()
{
    ROS_INFO("Initializing Publishers");
    controlinput_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1, true); 
    goalpos_pub_ = nh_.advertise<geometry_msgs::Pose2D>("/turtlebot/path", 1000, true);
}

void GotoGoal_W_Sim::currentposCallback(const nav_msgs::Odometry& odom) 
{   
    currentpos.x = odom.pose.pose.position.x;
    currentpos.y = odom.pose.pose.position.y;
    currentpos.theta = quatoeuler_yaw(odom);

    // distance between goal and robot in x-direction
    u_x = goalpos.x - currentpos.x;
    // distance between goal and robot in y-direction
    u_y = goalpos.y - currentpos.y;
    // angle from robot to goal
    double theta_g = atan2(u_y, u_x);
    // error between the goal angle and robot's angle
    u_angle = theta_g - currentpos.theta;

    // cout << "theta_g: " << theta_g << " theta: " << currentpos.theta << endl;

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
    else if (w < -1.5)
    {
        w = -1.5;
    }

    controlinput.angular.z = w;
    controlinput.linear.x = v;
    controlinput_pub_.publish(controlinput); //output the square of the received value;
    goalpos_pub_.publish(goalpos);
    // ros::Duration(0.01).sleep(); 
}

void GotoGoal_W_Sim::stopCallback(const nav_msgs::Odometry& odom) 
{   
    currentpos.x = odom.pose.pose.position.x;
    currentpos.y = odom.pose.pose.position.y;
    double theta = quatoeuler_yaw(odom);
    cout << "Xc pose: " << currentpos.x << " Xg pose: " << goalpos.x << " Yc pose: " << currentpos.y << " Yg pose: "<< goalpos.y <<endl;
    if ( ( abs(currentpos.x - goalpos.x)<0.05 ) && ( abs(currentpos.y - goalpos.y)<0.05 ) && (currentpos.x * goalpos.x > 0.01))
    {   
        controlinput.angular.z = 0;
        controlinput.linear.x = 0;
        controlinput_pub_.publish(controlinput);
        ROS_INFO("Finished...");
        ros::shutdown();
    }
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "turtlebot_gotogoal_w_sim"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    GotoGoal_W_Sim gotogoal_w_sim(&nh);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("Initializing...");
    ros::spin();
    return 0;
} 

