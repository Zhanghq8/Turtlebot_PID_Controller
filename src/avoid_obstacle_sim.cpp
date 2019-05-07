#include "turtlebot_pid/avoid_obstacle_sim.h"

Avoid_Obstacle_Sim::Avoid_Obstacle_Sim(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("In class constructor of Avoid_Obstacle_Sim");
    initSub(); // package up the messy work of creating subscribers; do this overhead in constructor
    initPub();
    setvelocity();
    setgoalpos();
    setpidgains();

}

void Avoid_Obstacle_Sim::setpidgains(double p, double i, double d)
{
    k_p = p;
    k_i = i;
    k_d = d;
}

void Avoid_Obstacle_Sim::setvelocity(double x) 
{
    v = x;
}

void Avoid_Obstacle_Sim::setgoalpos(double x, double y)
{
    goalpos.x = x;
    goalpos.y = y;
}

double Avoid_Obstacle_Sim::quatoeuler_yaw(const nav_msgs::Odometry& odom)
{
    tf::Quaternion qua(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    tf::Matrix3x3 mat(qua);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    return yaw;
}

void Avoid_Obstacle_Sim::initSub()
{
    ROS_INFO("Initializing Subscribers");  
    currentpos_sub_ = nh_.subscribe("/odom", 1, &Avoid_Obstacle_Sim::currentposCallback,this);
    laserpos_sub_ =  nh_.subscribe("/scan", 1, &Avoid_Obstacle_Sim::laserCallback,this);
    // stop_sub_ = nh_.subscribe("/odom", 1, &Avoid_Obstacle_Sim::stopCallback,this);
}


//member helper function to set up publishers;
void Avoid_Obstacle_Sim::initPub()
{
    ROS_INFO("Initializing Publishers");
    controlinput_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1, true); 
    goalpos_pub_ = nh_.advertise<geometry_msgs::Pose2D>("/turtlebot/path", 1000, true);
}


void Avoid_Obstacle_Sim::laserCallback(const sensor_msgs::LaserScan& scan)
{   
    double laserdis[5];
    // laser_angle[0] = scan.angle_min;
    // laser_angle[4] = scan.angle_max;
    // laser_angle[1] = scan.angle_min+160.0/640*(scan.angle_max-scan.angle_min);
    // laser_angle[2] = scan.angle_min+320.0/640*(scan.angle_max-scan.angle_min);
    // laser_angle[3] = scan.angle_min+480.0/640*(scan.angle_max-scan.angle_min);
    double laser_angle[5] = {-0.521568, -0.260107, 0.00135422, 0.262815, 0.524276};

    int laser_index[5] = {0,159, 319, 479, 639};
    for (int i=0; i<5; i++)
    {   
        if (scan.ranges[laser_index[i]] != scan.ranges[laser_index[i]])
        {
            laserdis[i] = scan.range_max;
        }
        else
        {
            laserdis[i] = scan.ranges[laser_index[i]];
        }
    }
    
    for (int i=0; i<5; i++)
    {   
        double x_or = laserdis[i]*cos(laser_angle[i]);
        double y_or = laserdis[i]*sin(laser_angle[i]);
        obstacle_pos[i][0] = currentpos.x + cos(currentpos.theta)*x_or - sin(currentpos.theta)*y_or;
        obstacle_pos[i][1] = currentpos.y + sin(currentpos.theta)*x_or + cos(currentpos.theta)*y_or;
    }
}


void Avoid_Obstacle_Sim::currentposCallback(const nav_msgs::Odometry& odom) 
{   
    currentpos.x = odom.pose.pose.position.x;
    currentpos.y = odom.pose.pose.position.y;
    currentpos.theta = quatoeuler_yaw(odom);

    geometry_msgs::Pose2D u_ao;
    u_ao.x = 0;
    u_ao.y = 0;

    for (int i=0; i<5; i++)
    {   
        // cout << i << ": " << obstacle_pos[i][0] << ", " << obstacle_pos[i][1] << " ";
        u_ao.x += (obstacle_pos[i][0] - currentpos.x) * lasergain[i];
        u_ao.y += (obstacle_pos[i][1] - currentpos.y) * lasergain[i];
    }
    // cout << "\n";
    // cout << u_ao.x << " " << u_ao.y << endl;

    // Compute the heading and error for the PID controller
    double theta_ao = atan2(u_ao.y, u_ao.x);
    
    double e_theta = theta_ao - currentpos.theta;

    e_k = atan2(sin(e_theta),cos(e_theta));

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
    cout << "w: " << w << endl;
    controlinput.angular.z = w;
    controlinput.linear.x = v;
    controlinput_pub_.publish(controlinput); //output the square of the received value;
    // goalpos_pub_.publish(goalpos);
    // ros::Duration(0.01).sleep(); 
}

void Avoid_Obstacle_Sim::stopCallback(const nav_msgs::Odometry& odom) 
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
    ros::init(argc, argv, "turtlebot_avoid_obstacle_sim"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    Avoid_Obstacle_Sim avoid_obstacle_sim(&nh);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("Initializing...");
    ros::spin();
    return 0;
} 

/*
angle_min: -0.521568
angle_max: 0.524276
angle_increment: 0.00163669
time_increment: 0
scan_time: 0
range_min: 0.45
range_max: 10
*/