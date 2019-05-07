#include "turtlebot_pid/avoid_obstacle_sim.h"

Avoid_Obstacle_Sim::Avoid_Obstacle_Sim(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("In class constructor of Avoid_Obstacle_Sim");
    initSub(); // package up the messy work of creating subscribers; do this overhead in constructor
    initPub();
    setvelocity();
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
    v_normal = x;
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
    laserpos_sub_ =  nh_.subscribe("/scan", 1, &Avoid_Obstacle_Sim::laserCallback,this);
    currentpos_sub_ = nh_.subscribe("/odom", 1, &Avoid_Obstacle_Sim::currentposCallback,this);
}


//member helper function to set up publishers;
void Avoid_Obstacle_Sim::initPub()
{
    ROS_INFO("Initializing Publishers");
    controlinput_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1, true); 
}


void Avoid_Obstacle_Sim::laserCallback(const sensor_msgs::LaserScan& scan)
{   
    cout << scan.range_min << endl;
    // laser_angle[0] = scan.angle_min;
    // laser_angle[4] = scan.angle_max;
    // laser_angle[1] = scan.angle_min+160.0/640*(scan.angle_max-scan.angle_min);
    // laser_angle[2] = scan.angle_min+320.0/640*(scan.angle_max-scan.angle_min);
    // laser_angle[3] = scan.angle_min+480.0/640*(scan.angle_max-scan.angle_min);
    double laser_angle[5] = {-0.521568, -0.260107, 0.00135422, 0.262815, 0.524276};

    int laser_index[5] = {0, 159, 319, 479, 639};
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
    if (laserdis[2] < 1.0)
    {
        w = -0.8;
        v_ao = 0.1;
        controlinput.angular.z = w;
        controlinput.linear.x = v_ao;
        controlinput_pub_.publish(controlinput);

    }
    else
    {
        currentpos.x = odom.pose.pose.position.x;
        currentpos.y = odom.pose.pose.position.y;
        currentpos.theta = quatoeuler_yaw(odom);

        geometry_msgs::Pose2D u_ao;
        u_ao.x = 0;
        u_ao.y = 0;

        for (int i=0; i<5; i++)
        {   
            u_ao.x += (obstacle_pos[i][0] - currentpos.x) * lasergain[i];
            u_ao.y += (obstacle_pos[i][1] - currentpos.y) * lasergain[i];
        }

        // Compute the heading and error for the PID controller
        double theta_ao = atan2(u_ao.y, u_ao.x);
        
        double e_theta = theta_ao - currentpos.theta;
        cout << currentpos.theta-theta_ao << endl;

        // cout << "Theta: " << theta_ao << ", " << currentpos.theta << " \n";
        
        e_k = atan2(sin(e_theta),cos(e_theta));

        // error for the proportional term
        e_P = e_k;
        // error for the integral term. Approximate the integral using the accumulated error, obj.E_k, and the error for this time step
        e_I = e_k + E_k;

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
        controlinput.linear.x = v_normal;
        controlinput_pub_.publish(controlinput); // 
    }

    cout << "w: " << w << endl;
 
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