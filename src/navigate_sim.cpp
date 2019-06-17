#include "turtlebot_pid/navigate_sim.h"

Navigate_Sim::Navigate_Sim(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("In class constructor of Navigate_Sim");
    initSub(); // package up the messy work of creating subscribers; do this overhead in constructor
    initPub();
    setgoalpos();
    setvelocity();
    setpidgains();
    
}

void Navigate_Sim::setgoalpos(double x, double y)
{
    goalpos.x = x;
    goalpos.y = y;
}

void Navigate_Sim::setpidgains(double p, double i, double d)
{
    k_p = p;
    k_i = i;
    k_d = d;
}

void Navigate_Sim::setvelocity(double x, double y) 
{
    v_normal = x;
    v_ao = y;
}

double Navigate_Sim::quatoeuler_yaw(const nav_msgs::Odometry& odom)
{
    tf::Quaternion qua(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    tf::Matrix3x3 mat(qua);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    return yaw;
}

void Navigate_Sim::initSub()
{
    ROS_INFO("Initializing Subscribers");  
    laserpos_sub_ =  nh_.subscribe("/scan", 1, &Navigate_Sim::laserCallback,this);
    currentpos_sub_ = nh_.subscribe("/odom", 1, &Navigate_Sim::currentposCallback,this);
    stop_sub_ = nh_.subscribe("/odom", 1, &Navigate_Sim::stopCallback,this);
}


//member helper function to set up publishers;
void Navigate_Sim::initPub()
{
    ROS_INFO("Initializing Publishers");
    controlinput_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1, true); 
}


void Navigate_Sim::laserCallback(const sensor_msgs::LaserScan& scan)
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
        if (scan.ranges[laser_index[i]] == scan.ranges[laser_index[i]] && scan.ranges[laser_index[i]] > 0.6)
        {
            laserdis[i] = scan.ranges[laser_index[i]];
        }

        else if (scan.ranges[laser_index[i]] - 0.6 <= 0)
        {
            laserdis[i] = scan.range_min;
        }
        else 
        {
            laserdis[i] = scan.range_max;
        }
        cout << "Laser: " << i << ": " << laserdis[i] << " . ";
    }
    cout << " \n";
    
    for (int i=0; i<5; i++)
    {   
        double x_or = laserdis[i]*cos(laser_angle[i]);
        double y_or = laserdis[i]*sin(laser_angle[i]);
        obstacle_pos[i][0] = currentpos.x + cos(currentpos.theta)*x_or - sin(currentpos.theta)*y_or;
        obstacle_pos[i][1] = currentpos.y + sin(currentpos.theta)*x_or + cos(currentpos.theta)*y_or;
        // cout << i << ": " << obstacle_pos[i][0] << ", " << obstacle_pos[i][1] << endl;
    }
}


void Navigate_Sim::currentposCallback(const nav_msgs::Odometry& odom) 
{   
    double min_laserdis = laserdis[0];
    for (int i=0; i<5; i++) 
    {   
        if (laserdis[i] < min_laserdis) {
            min_laserdis = laserdis[i];
        }
    }

    // cout << "aa laser" << min_laserdis << endl;

    // get current state of robot u_gtg
    // cout << "Controller!!" << endl;
    currentpos.x = odom.pose.pose.position.x;
    currentpos.y = odom.pose.pose.position.y;
    currentpos.theta = quatoeuler_yaw(odom);
    // cout << "goalpos: " << goalpos.x << " " << goalpos.y << endl;

    geometry_msgs::Pose2D u_gtg, u_ao;

    // Go to goal vector
    u_gtg.x = goalpos.x - currentpos.x;
    u_gtg.y = goalpos.y - currentpos.y;

    // Avoid obstacle vector u_ao
    for (int i=0; i<5; i++)
    {   
        u_ao.x += (obstacle_pos[i][0] - currentpos.x) * lasergain[i];
        u_ao.y += (obstacle_pos[i][1] - currentpos.y) * lasergain[i];
    }

    // Follow wall clockwise vector u_fw_c
    geometry_msgs::Pose2D u_fw_c, u_fw_c_t, u_fw_c_t1, u_fw_c_t2;
 
    u_fw_c_t1.x = obstacle_pos[4][0];
    u_fw_c_t1.y = obstacle_pos[4][1];
    u_fw_c_t2.x = obstacle_pos[3][0];
    u_fw_c_t2.y = obstacle_pos[3][1];
    u_fw_c_t.x = u_fw_c_t2.x - u_fw_c_t1.x;
    u_fw_c_t.y = u_fw_c_t2.y - u_fw_c_t1.y;

    geometry_msgs::Pose2D u_fw_c_t_norm, u_c_avoid;

    // calculate the norm of u_fw_c_t vector;
    u_fw_c_t_norm.x = u_fw_c_t.x/ (sqrt(pow(u_fw_c_t.x, 2)+pow(u_fw_c_t.y, 2)) + 0.0001);
    u_fw_c_t_norm.y = u_fw_c_t.y/ (sqrt(pow(u_fw_c_t.x, 2)+pow(u_fw_c_t.y, 2))) + 0.0001;

    u_c_avoid = u_fw_c_t1;
    
    geometry_msgs::Pose2D u_fw_c_p, u_fw_c_p_norm;
    // ufw_p: vector points from the robot to the closest point on ufw
    u_fw_c_p.x = (u_c_avoid.x - currentpos.x) - ((u_c_avoid.x - currentpos.x)*u_fw_c_t_norm.x)*u_fw_c_t_norm.x;
    u_fw_c_p.y = (u_c_avoid.y - currentpos.y) - ((u_c_avoid.y - currentpos.y)*u_fw_c_t_norm.y)*u_fw_c_t_norm.y;

    // calculate the norm of u_fw_cp vector;
    u_fw_c_p_norm.x = u_fw_c_p.x/ (sqrt(pow(u_fw_c_p.x, 2)+pow(u_fw_c_p.y, 2)) + 0.0001);
    u_fw_c_p_norm.y = u_fw_c_p.y/ (sqrt(pow(u_fw_c_p.x, 2)+pow(u_fw_c_p.y, 2))) + 0.0001;

    // u_fw_c: vector points towards the obstacle when the distance to the obstacle
    u_fw_c.x = diswall * u_fw_c_t_norm.x + (u_fw_c_p.x - diswall * u_fw_c_p_norm.x);
    u_fw_c.y = diswall * u_fw_c_t_norm.y + (u_fw_c_p.y - diswall * u_fw_c_p_norm.y);

    // Follow wall counter-clockwise vector u_fw_cc

    geometry_msgs::Pose2D u_fw_cc, u_fw_cc_t, u_fw_cc_t1, u_fw_cc_t2;
 
    u_fw_cc_t1.x = obstacle_pos[0][0];
    u_fw_cc_t1.y = obstacle_pos[0][1];
    u_fw_cc_t2.x = obstacle_pos[1][0];
    u_fw_cc_t2.y = obstacle_pos[1][1];
    u_fw_cc_t.x = u_fw_cc_t2.x - u_fw_cc_t1.x;
    u_fw_cc_t.y = u_fw_cc_t2.y - u_fw_cc_t1.y;

    geometry_msgs::Pose2D u_fw_cc_t_norm, u_cc_avoid;

    // calculate the norm of u_fw_cc_t vector;
    u_fw_cc_t_norm.x = u_fw_cc_t.x/ (sqrt(pow(u_fw_cc_t.x, 2)+pow(u_fw_cc_t.y, 2)) + 0.0001);
    u_fw_cc_t_norm.y = u_fw_cc_t.y/ (sqrt(pow(u_fw_cc_t.x, 2)+pow(u_fw_cc_t.y, 2))) + 0.0001;

    u_cc_avoid = u_fw_cc_t1;
    
    geometry_msgs::Pose2D u_fw_cc_p, u_fw_cc_p_norm;
    // ufw_p: vector points from the robot to the closest point on ufw
    u_fw_cc_p.x = (u_cc_avoid.x - currentpos.x) - ((u_cc_avoid.x - currentpos.x)*u_fw_cc_t_norm.x)*u_fw_cc_t_norm.x;
    u_fw_cc_p.y = (u_cc_avoid.y - currentpos.y) - ((u_cc_avoid.y - currentpos.y)*u_fw_cc_t_norm.y)*u_fw_cc_t_norm.y;

    // calculate the norm of u_fw_ccp vector;
    u_fw_cc_p_norm.x = u_fw_cc_p.x/ (sqrt(pow(u_fw_cc_p.x, 2)+pow(u_fw_cc_p.y, 2)) + 0.0001);
    u_fw_cc_p_norm.y = u_fw_cc_p.y/ (sqrt(pow(u_fw_cc_p.x, 2)+pow(u_fw_cc_p.y, 2))) + 0.0001;

    // u_fw_cc: vector points towards the obstacle when the distance to the obstacle
    u_fw_cc.x = diswall * u_fw_cc_t_norm.x + (u_fw_cc_p.x - diswall * u_fw_cc_p_norm.x);
    u_fw_cc.y = diswall * u_fw_cc_t_norm.y + (u_fw_cc_p.y - diswall * u_fw_cc_p_norm.y);

    // cout << count << endl;
    // cout << "u_gtg" << u_gtg.x << "  " << u_gtg.y << endl;
    // cout << "u_ao" << u_ao.x << "  " << u_ao.y << endl;
    // cout << "u_fw_c" << u_fw_c.x << "  " << u_fw_c.y << endl;
    // cout << "u_fw_cc" << u_fw_cc.x << "  " << u_fw_cc.y << endl;

    // cout << "gtgfw " << distance_gtg_fw << endl;
    // cout << "check ugtg ao " << u_gtg.x * u_ao.x + u_gtg.y * u_ao.y << endl;
    // cout << "ufwc " << u_gtg.x * u_fw_c.x + u_gtg.y * u_fw_c.y << endl;
    // cout << "ufwcc " << u_gtg.x * u_fw_cc.x + u_gtg.y * u_fw_cc.y << endl;

    if (min_laserdis < distance_gtg_fw)// && u_gtg.x * u_ao.x + u_gtg.y * u_ao.y > 0) 
    {
        u_nav = u_gtg;
        state_last = state_current;
        state_current = "Go to goal";
        count = 0;
    }

    if (abs(min_laserdis - diswall) <= 0.20 && u_gtg.x * u_fw_c.x + u_gtg.y * u_fw_c.y > 0)// ||\
         (abs(atan2(u_fw_c.y, u_fw_c.x) - currentpos.theta) < 0.02 && abs(min_laserdis - diswall) <= 0.08)) 
    {   
        if (count == 0) {
            distance_gtg_fw = sqrt(pow(u_gtg.x, 2)+pow(u_gtg.y, 2));
        }
        u_nav = u_fw_c;
        state_last = state_current;
        state_current = "Follow wall clockwise";
        count += 0.00001;
    }

    if (abs(min_laserdis - diswall) <= 0.20 && u_gtg.x * u_fw_cc.x + u_gtg.y * u_fw_cc.y > 0)// ||\
         (abs(atan2(u_fw_cc.y, u_fw_cc.x) - currentpos.theta) < 0.02 && abs(min_laserdis - diswall) <= 0.20)) 
    {
        if (count == 0) {
            distance_gtg_fw = sqrt(pow(u_gtg.x, 2)+pow(u_gtg.y, 2));
        }
        u_nav = u_fw_cc;
        state_last = state_current;
        state_current = "Follow wall counter-clockwise";
        count += 0.00001;
    }

    if (min_laserdis < distance_safe) 
    {   
        u_nav = u_ao;
        state_last = state_current;
        state_current = "Avoid obstacle"; 
        count = 0;      
    }

    cout << state_current << endl;

    // if (state_current == "Go to goal")
    // {
    //     u_nav = u_gtg;
    // }
    // else if (state_current == "Avoid obstacle")
    // {
    //     u_nav = u_ao;
    // }

    // else if (state_current == "Follow wall counter-clockwise") 
    // {
    //     u_nav = u_fw_cc;
    // }
    // else if (state_current == "Follow wall clockwise")
    // {
    //     u_nav = u_fw_c;
    // }

    u_nav.theta = atan2(u_nav.y, u_nav.x);
    
    double e_theta = u_nav.theta - currentpos.theta;
    
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

    if (w > 1.0)
    {
        w = 1.0;
    }
    else if (w < -1.0)
    {
        w = -1.0;
    }

    if (min_laserdis < 0.6) 
    {
        controlinput.linear.x = v_ao;
    }
    else 
    {
        controlinput.linear.x = v_normal;
    }

    controlinput.angular.z = w;
    controlinput_pub_.publish(controlinput);
}

void Navigate_Sim::stopCallback(const nav_msgs::Odometry& odom) 
{   
    currentpos.x = odom.pose.pose.position.x;
    currentpos.y = odom.pose.pose.position.y;
    double theta = quatoeuler_yaw(odom);
    // cout << "Xc pose: " << currentpos.x << " Xg pose: " << goalpos.x << " Yc pose: " << currentpos.y << " Yg pose: "<< goalpos.y <<endl;
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
    ros::init(argc, argv, "turtlebot_navigate_Sim"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    Navigate_Sim navigate_sim(&nh);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("Initializing...");
    ros::spin();
    return 0;
} 