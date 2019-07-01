#include "turtlebot_pid/followwall_sim.h"

Follow_Wall_Sim::Follow_Wall_Sim(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("In class constructor of Follow_Wall_Sim");
    setdirection(1);
    initSub(); // package up the messy work of creating subscribers; do this overhead in constructor
    initPub();
    setvelocity();
    setpidgains();

    w = 0;
    // error dynamics
    e_P = 0;
    e_I = 0;
    e_D= 0;

    // accumulated error
    E_k = 0;
    // previous error
    e_k_previous = 0;

    //distance to the wall 
    diswall = 0.70;
}

void Follow_Wall_Sim::setpidgains(double p, double i, double d)
{
    k_p = p;
    k_i = i;
    k_d = d;
}

void Follow_Wall_Sim::setvelocity(double x, double y) 
{
    v_normal = x;
    v_ao = y;
}

void Follow_Wall_Sim::setdirection(int x) 
{
    direction = x;
}

double Follow_Wall_Sim::quatoeuler_yaw(const nav_msgs::Odometry& odom)
{
    tf::Quaternion qua(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    tf::Matrix3x3 mat(qua);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    return yaw;
}

void Follow_Wall_Sim::initSub()
{
    ROS_INFO("Initializing Subscribers");  
    laserpos_sub_ =  nh_.subscribe("/scan", 1, &Follow_Wall_Sim::laserCallback,this);
    currentpos_sub_ = nh_.subscribe("/odom", 1, &Follow_Wall_Sim::currentposCallback,this);
}


//member helper function to set up publishers;
void Follow_Wall_Sim::initPub()
{
    ROS_INFO("Initializing Publishers");
    controlinput_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1, true); 
}


void Follow_Wall_Sim::laserCallback(const sensor_msgs::LaserScan& scan)
{   
    // laser_angle[0] = scan.angle_min;
    // laser_angle[4] = scan.angle_max;
    // laser_angle[1] = scan.angle_min+160.0/640*(scan.angle_max-scan.angle_min);
    // laser_angle[2] = scan.angle_min+320.0/640*(scan.angle_max-scan.angle_min);
    // laser_angle[3] = scan.angle_min+480.0/640*(scan.angle_max-scan.angle_min);
    double laser_angle[5] = {-0.521568, -0.260107, 0.00135422, 0.262815, 0.524276};

    int laser_index[5] = {0, 159, 319, 479, 639};
    for (int i=0; i<5; i++)
    {   
        if (scan.ranges[laser_index[i]] == scan.ranges[laser_index[i]] && scan.ranges[laser_index[i]] > 0.8)
        {
            laserdis[i] = scan.ranges[laser_index[i]];
        }

        else if (scan.ranges[laser_index[i]] - 1.0 <= 0)
        {
            laserdis[i] = scan.range_min;
        }
        else 
        {
            laserdis[i] = scan.range_max;
        }
    }
    // cout << " \n";
    
    for (int i=0; i<5; i++)
    {   
        double x_or = laserdis[i]*cos(laser_angle[i]);
        double y_or = laserdis[i]*sin(laser_angle[i]);
        obstacle_pos[i][0] = currentpos.x + cos(currentpos.theta)*x_or - sin(currentpos.theta)*y_or;
        obstacle_pos[i][1] = currentpos.y + sin(currentpos.theta)*x_or + cos(currentpos.theta)*y_or;
        cout << i << ": " << obstacle_pos[i][0] << ", " << obstacle_pos[i][1] << endl;
    }
}


void Follow_Wall_Sim::currentposCallback(const nav_msgs::Odometry& odom) 
{   
    double min_laserdis = laserdis[0];
    for (int i=0; i<5; i++) 
    {   
        if (laserdis[i] < min_laserdis) {
            min_laserdis = laserdis[i];
        }
    }

    // get current state of robot
    cout << "Controller!!" << endl;
    currentpos.x = odom.pose.pose.position.x;
    currentpos.y = odom.pose.pose.position.y;
    currentpos.theta = quatoeuler_yaw(odom);

    geometry_msgs::Pose2D u_fw, u_fw_t, u_fw_t1, u_fw_t2;

    // find two index of smallest laser data from 0-2(left) or 2-4(right) 
    switch (direction)
    {
        case 1:
        u_fw_t1.x = obstacle_pos[0][0];
        u_fw_t1.y = obstacle_pos[0][1];
        u_fw_t2.x = obstacle_pos[1][0];
        u_fw_t2.y = obstacle_pos[1][1];
        break; 

        case -1:
        u_fw_t1.x = obstacle_pos[4][0];
        u_fw_t1.y = obstacle_pos[4][1];
        u_fw_t2.x = obstacle_pos[3][0];
        u_fw_t2.y = obstacle_pos[3][1];
        break; 
    }

    // get wall vector
    u_fw_t.x = u_fw_t2.x - u_fw_t1.x;
    u_fw_t.y = u_fw_t2.y - u_fw_t1.y;

    geometry_msgs::Pose2D u_fw_t_norm, u_avoid, u_robot;

    // calculate the norm of u_fw_t vector;
    u_fw_t_norm.x = u_fw_t.x/ (sqrt(pow(u_fw_t.x, 2)+pow(u_fw_t.y, 2)) + 0.0001);
    u_fw_t_norm.y = u_fw_t.y/ (sqrt(pow(u_fw_t.x, 2)+pow(u_fw_t.y, 2))) + 0.0001;

    u_avoid = u_fw_t1;
    u_robot = currentpos;

    geometry_msgs::Pose2D u_fw_p, u_fw_p_norm;
    // ufw_p: vector points from the robot to the closest point on ufw
    u_fw_p.x = (u_avoid.x - u_robot.x) - ((u_avoid.x - u_robot.x)*u_fw_t_norm.x)*u_fw_t_norm.x;
    u_fw_p.y = (u_avoid.y - u_robot.y) - ((u_avoid.y - u_robot.y)*u_fw_t_norm.y)*u_fw_t_norm.y;

    // calculate the norm of u_fwp vector;
    u_fw_p_norm.x = u_fw_p.x/ (sqrt(pow(u_fw_p.x, 2)+pow(u_fw_p.y, 2)) + 0.0001);
    u_fw_p_norm.y = u_fw_p.y/ (sqrt(pow(u_fw_p.x, 2)+pow(u_fw_p.y, 2))) + 0.0001;

    // u_fw: vector points towards the obstacle when the distance to the obstacle
    u_fw.x = diswall * u_fw_t_norm.x + (u_fw_p.x - diswall * u_fw_p_norm.x);
    u_fw.y = diswall * u_fw_t_norm.y + (u_fw_p.y - diswall * u_fw_p_norm.y);

    u_fw.theta = atan2(u_fw.y, u_fw.x);
    
    double e_theta = u_fw.theta - currentpos.theta;
    
    e_k = atan2(sin(e_theta),cos(e_theta));

    // error for the proportional term
    e_P = e_k;
    // error for the integral term. Approximate the integral using the accumulated error, E_k, and the error for this time step
    e_I = e_k + E_k;

    // error for the derivative term. Hint: Approximate the derivative using the previous error, and the error for this time step, e_k.
    e_D = e_k - e_k_previous; 
    // update errors
    E_k = e_I;
    e_k_previous = e_k;

    // control input 
    w = k_p*e_P + k_i*e_I + k_d*e_D;

    if (min_laserdis < 0.8) {
        if (w > 1.0)
        {
            w = 1.0;
        }
        else if (w < -1.0)
        {
            w = -1.0;
        }
        controlinput.linear.x = v_ao;
        controlinput.angular.z = w;
    }
    else {
        
        if (w > 1.0)
        {
            w = 1.0;
        }
        else if (w < -1.0)
        {
            w = -1.0;
        }
        controlinput.angular.z = w;
        controlinput.linear.x = v_normal;
    }
    controlinput_pub_.publish(controlinput);
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "turtlebot_follow_wall_Sim"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    Follow_Wall_Sim follow_wall_sim(&nh);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("Initializing...");
    ros::spin();
    return 0;
} 