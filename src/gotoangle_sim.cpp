#include "turtlebot_pid/gotoangle_sim.h"

GotoAngle_Sim::GotoAngle_Sim(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("In class constructor of GotoAngle_Sim");
    initSub(); // package up the messy work of creating subscribers; do this overhead in constructor
    initPub();
    setvelocity();
    //-pi-pi
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

void GotoAngle_Sim::setpidgains(double p, double i, double d)
{
    k_p = p;
    k_i = i;
    k_d = d;
}

void GotoAngle_Sim::setvelocity(double x) 
{
    v = x;
}

void GotoAngle_Sim::setgoalpos(double x)
{
    theta_d = x;
}

double GotoAngle_Sim::quatoeuler_yaw(const nav_msgs::Odometry& odom)
{
    tf::Quaternion qua(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    tf::Matrix3x3 mat(qua);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    return yaw;
    }


void GotoAngle_Sim::initSub()
{
    ROS_INFO("Initializing Subscribers"); 
    currentpos_sub_ = nh_.subscribe("/odom", 1, &GotoAngle_Sim::currentposCallback,this); 
    stop_sub_ = nh_.subscribe("/odom", 1, &GotoAngle_Sim::stopCallback,this);
}


//member helper function to set up publishers;
void GotoAngle_Sim::initPub()
{
    ROS_INFO("Initializing Publishers");
    controlinput_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1, true); 
}

void GotoAngle_Sim::currentposCallback(const nav_msgs::Odometry& odom) 
{   
    double theta = quatoeuler_yaw(odom);
    currentpos.x = odom.pose.pose.position.x;
    currentpos.y = odom.pose.pose.position.y;
    // currentpos.theta = odom.pose.pose.orientation.z;

    e_k = theta_d-theta;
    // use atan2 to make e_k in (-pi,pi)
    e_k = atan2(sin(e_k), cos(e_k));

    // error for the proportional term
    e_P = e_k;

    // error for the integral term. Approximate the integral using the accumulated error, obj.E_k, and the error for this time step
    e_I = e_k + E_k;

    // error for the derivative term. Hint: Approximate the derivative using the previous error, and the error for this time step, e_k.
    e_D = e_k - e_k_previous; 
    // update errors
    E_k = e_I;
    e_k_previous = e_k;
    // cout << e_k << " "<< e_I << " "<< E_k << endl;

    // control input 
    w = k_p*e_P + k_i*e_I + k_d*e_D;

    if (w > 1.0)
    {
        w = 1.0;
    }
    else if (w < -1.0)
    {
        w = -1.0;
    }

    cout << "****" << w<< endl;
    //************
    controlinput.angular.z = w;
    controlinput.linear.x = v;
    controlinput_pub_.publish(controlinput); //output the square of the received value; 
}


void GotoAngle_Sim::stopCallback(const nav_msgs::Odometry& odom) 
{   
    currentpos.x = odom.pose.pose.position.x;
    currentpos.y = odom.pose.pose.position.y;
    double theta = quatoeuler_yaw(odom);
    if ( ( abs(theta - theta_d)<0.02 ) && (theta * theta_d > 0.01))
    {
        ROS_INFO("Finished...");
        ros::shutdown();
    }
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "turtlebot_gotoangle_sim"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    GotoAngle_Sim gotoangle_sim(&nh);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("Initializing...");
    ros::spin();
    return 0;
} 

