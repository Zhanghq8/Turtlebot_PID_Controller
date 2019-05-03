#include "turtlebot_pid/gotogoal.h"

//get the goal pos 

//calculate the heading angle

//get current pos of the turtlebot

//stop turtlebot when the goal is reached



GotoGoal::GotoGoal(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("In class constructor of GotoGoal");
    initSub(); // package up the messy work of creating subscribers; do this overhead in constructor
    initPub();
    
    //initialize variables here, as needed
}

void GotoGoal::initSub()
{
    ROS_INFO("Initializing Subscribers");
    // goalpos_sub_ = nh_.subscribe("/vicon/marker", 1, &GotoGoal::goalposCallback,this); 
    // currentpos_sub_ = nh_.subscribe("/vicon/marker", 1, &GotoGoal::currentposCallback,this);  
    currentpos_sub_ = nh_.subscribe("/odom", 1, &GotoGoal::currentposCallback,this); 
}


//member helper function to set up publishers;
void GotoGoal::initPub()
{
    ROS_INFO("Initializing Publishers");
    controlinput_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1, true); 
    // note: COULD make minimal_publisher_ a public member function, if want to use it within "main()"
}


double GotoGoal::heading_angle(double x, double y)
{
    return atan2(x, y);
}

// void GotoGoal::currentposCallback(const vicon_bridge::Marker& markers) 
void GotoGoal::currentposCallback(const nav_msgs::Odometry& odom) 
{   
/*
    goalpos.x = 1;
    goalpos.y = 0.3;
    currentpos.x = odom.pose.pose.position.x;
    currentpos.y = odom.pose.pose.position.y;
    currentpos.theta = heading_angle(currentpos.x, currentpos.y);

    // distance between goal and robot in x-direction
    u_x = goalpos.x - currentpos.x;
    // distance between goal and robot in y-direction
    u_y = goalpos.y - currentpos.y;
    // angle from robot to goal
    double theta_g = atan2(u_y, u_x);
    cout << u_x<< u_y<<theta_g << endl;
    // error between the goal angle and robot's angle
    u_angle = theta_g - currentpos.theta;
    double e_k = atan2(sin(u_angle),cos(u_angle));

    cout << e_k << endl;

    // error for the proportional term
    e_P = e_k;
    // error for the integral term. Approximate the integral using the accumulated error, obj.E_k, and the error for this time step
    e_I = e_k + E_k;

    cout << e_I << endl;
    // error for the derivative term. Hint: Approximate the derivative using the previous error, and the error for this time step, e_k.
    e_D = e_k - e_k_previous; 
    // update errors
    E_k = e_I;
    e_k_previous = e_k;

    // control input 
    double w = k_p*e_P + k_i*e_I + k_d*e_D;
    if (w > 0.5)
    {
        w = 0.5;
    }
    // cout << w << endl;
*/
    currentpos.x = odom.pose.pose.position.x;
    currentpos.y = odom.pose.pose.position.y;
    // currentpos.theta = heading_angle(currentpos.x, currentpos.y);
    currentpos.theta = odom.pose.pose.orientation.z;
    e_k = theta_d-currentpos.theta;

    e_k = atan2(sin(e_k), cos(e_k));
    
    double w = k_p*e_k;

    controlinput.angular.z = w;
    controlinput.linear.x = v;
    controlinput_pub_.publish(controlinput); //output the square of the received value; 
}



// void GotoGoal::goalposCallback(const vicon_bridge::Marker& markers) {

//     goalpos.x = ;
//     goalpos.y = ;
//     goalpos.theta = heading_angle(goalpos.x, goalpos.y);
// }

// void TurtlebotTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
// { 
//   geometry_msgs::Twist vel;
//   vel.angular.z = a_scale_*joy->axes[angular_];
//   vel.linear.x = l_scale_*joy->axes[linear_];
//   last_published_ = vel;
//   deadman_pressed_ = joy->buttons[deadman_axis_];
// }

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "turtlebot_gotogoal"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    GotoGoal gotogoal(&nh);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("Initializing...");
    ros::spin();
    return 0;
} 

