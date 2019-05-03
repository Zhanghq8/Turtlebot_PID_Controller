#include "turtlebot_pid/gotogoal_vw_sim.h"

//get the goal pos 

//calculate the heading angle

//get current pos of the turtlebot

//stop turtlebot when the goal is reached


GotoGoal_VW_Sim::GotoGoal_VW_Sim(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("In class constructor of GotoGoal_VW_Sim");
    initSub(); // package up the messy work of creating subscribers; do this overhead in constructor
    initPub();
    setgoalpos();
    setpidgains();

}

void GotoGoal_VW_Sim::setpidgains(double px, double ix, double dx, double py, double iy, double dy, double ptheta, double itheta, double dtheta)
{
    kp_x = px;
    ki_x = ix;
    kd_x = dx;
    kp_y = py;
    ki_y = iy;
    kd_y = dy;
    kp_theta = ptheta;
    ki_theta = itheta;
    kd_theta = dtheta;
}


void GotoGoal_VW_Sim::setgoalpos(double x, double y, double theta)
{
    goalpos.x = x;
    goalpos.y = y;
    goalpos.theta = theta;
}

void GotoGoal_VW_Sim::initSub()
{
    ROS_INFO("Initializing Subscribers");  
    currentpos_sub_ = nh_.subscribe("/odom", 1, &GotoGoal_VW_Sim::currentposCallback,this); 
    stop_sub_ = nh_.subscribe("/odom", 1, &GotoGoal_VW_Sim::stopCallback,this);
}


//member helper function to set up publishers;
void GotoGoal_VW_Sim::initPub()
{
    ROS_INFO("Initializing Publishers");
    controlinput_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1, true); 
}

void GotoGoal_VW_Sim::currentposCallback(const nav_msgs::Odometry& odom) 
{   
    currentpos.x = odom.pose.pose.position.x;
    currentpos.y = odom.pose.pose.position.y;
    currentpos.theta = odom.pose.pose.orientation.z;

    // error between goalpos and robotpos in x-direction
    u_x = goalpos.x - currentpos.x;
    // error between goalpos and robotpos in y-direction
    u_y = goalpos.y - currentpos.y;

    // error between the goal angle and robot's angle
    u_theta = goalpos.theta - currentpos.theta;
    
    ek_x = u_x;
    ek_y = u_y;
    ek_theta = atan2(sin(u_theta),cos(u_theta));

    // x pos
    // error for the proportional term
    eP_x = ek_x;
    // error for the integral term. Approximate the integral using the accumulated error, obj.E_k, and the error for this time step
    eI_x = ek_x + Ek_x;

    // error for the derivative term. Hint: Approximate the derivative using the previous error, and the error for this time step, e_k.
    eD_x = ek_x - ek_x_previous; 
    // update errors
    Ek_x = eI_x;
    ek_x_previous = ek_x;

    // y pos
    // error for the proportional term
    eP_y = ek_y;
    // error for the integral term. Approximate the integral using the accumulated error, obj.E_k, and the error for this time step
    eI_y = ek_y + Ek_y;

    // error for the derivative term. Hint: Approximate the derivative using the previous error, and the error for this time step, e_k.
    eD_y = ek_y - ek_y_previous; 
    // update errors
    Ek_y = eI_y;
    ek_y_previous = ek_y;

    // theta pos
    // error for the proportional term
    eP_theta = ek_theta;
    // error for the integral term. Approximate the integral using the accumulated error, obj.E_k, and the error for this time step
    eI_theta = ek_theta + Ek_theta;

    // error for the derivative term. Hint: Approximate the derivative using the previous error, and the error for this time step, e_k.
    eD_theta = ek_theta - ek_theta_previous; 
    // update errors
    Ek_theta = eI_theta;
    ek_theta_previous = ek_theta;

    vx = kp_x*eP_x + ki_x*eI_x + kd_x*eD_x;
    vy = kp_y*eP_y + ki_y*eI_y + kd_y*eD_y;


    // control input
    v = sqrt( pow(vx,2) + pow(vy,2) );
    w = kp_theta*eP_theta + ki_theta*eI_theta + kd_theta*eD_theta;

    if (v > 0.5)
    {
        v = 0.5;
    }
    else if (v < -0.5)
    {
        v = -0.5;
    }

    if (w > 1.5)
    {
        w = 1.5;
    }
    else if (w < -1.5)
    {
        w = -1.5;
    }
    // cout << w << endl;

    controlinput.angular.z = w;
    controlinput.linear.x = v;
    cout << "V input: " << v << " W input: " << w << endl;
    controlinput_pub_.publish(controlinput); //output the square of the received value; 
}

void GotoGoal_VW_Sim::stopCallback(const nav_msgs::Odometry& odom) 
{   
    currentpos.x = odom.pose.pose.position.x;
    currentpos.y = odom.pose.pose.position.y;
    currentpos.theta = odom.pose.pose.orientation.z;
    cout << "X pose: "<< currentpos.x << ". Y pose: "<< currentpos.y << " Theta pose: " << currentpos.theta <<endl;
    if ( ( abs(currentpos.x - goalpos.x)<0.1 ) && ( abs(currentpos.y - goalpos.y)<0.1 ) && ( abs(currentpos.theta - goalpos.theta)<0.1 ) && (currentpos.x * goalpos.x > 0.01))
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
    ros::init(argc, argv, "turtlebot_gotogoal_vw_sim"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    GotoGoal_VW_Sim gotogoal_vw_sim(&nh);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("Initializing...");
    ros::spin();
    return 0;
} 

