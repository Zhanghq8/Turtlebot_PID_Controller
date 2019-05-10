#include "turtlebot_pid/GTG_AO_sim.h"

GTG_AO_Sim::GTG_AO_Sim(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("In class constructor of GTG_AO_Sim");
    initSub(); // package up the messy work of creating subscribers; do this overhead in constructor
    initPub();
    setgoalpos();
    setvelocity();
    setpidgains();

}

void GTG_AO_Sim::setpidgains(double p, double i, double d)
{
    k_p = p;
    k_i = i;
    k_d = d;
}

void GTG_AO_Sim::setvelocity(double x) 
{
    v_normal = x;
}

void GTG_AO_Sim::setgoalpos(double x, double y)
{
    goalpos.x = x;
    goalpos.y = y;
}

double GTG_AO_Sim::quatoeuler_yaw(const nav_msgs::Odometry& odom)
{
    tf::Quaternion qua(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    tf::Matrix3x3 mat(qua);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    return yaw;
}

void GTG_AO_Sim::initSub()
{
    ROS_INFO("Initializing Subscribers");  
    laserpos_sub_ =  nh_.subscribe("/scan", 1, &GTG_AO_Sim::laserCallback,this);
    currentpos_sub_ = nh_.subscribe("/odom", 1, &GTG_AO_Sim::currentposCallback,this);
    stop_sub_ = nh_.subscribe("/odom", 1, &GTG_AO_Sim::stopCallback,this);
}


//member helper function to set up publishers;
void GTG_AO_Sim::initPub()
{
    ROS_INFO("Initializing Publishers");
    controlinput_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1, true); 
}


void GTG_AO_Sim::laserCallback(const sensor_msgs::LaserScan& scan)
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


void GTG_AO_Sim::currentposCallback(const nav_msgs::Odometry& odom) 
{   
    if (laserdis[3] < 0.8)
    {
        w = 0.8;
        v_ao = 0.1;
        controlinput.angular.z = w;
        controlinput.linear.x = v_ao;
        controlinput_pub_.publish(controlinput);
    }
    else if (laserdis[1] < 0.8)
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

        //avoid obstacle vector, go to goal vector
        //normalized avoid obstacle vector, go to goal vector
        //blending vector combine gotogoal and avoid obstacle
        geometry_msgs::Pose2D u_ao, u_gtg, u_ao_n, u_gtg_n, u_ao_gtg;
        u_ao.x = 0;
        u_ao.y = 0;
        u_ao.theta = 0;

        u_gtg.x = 0;
        u_gtg.y = 0;
        u_gtg.theta = 0;

        // distance between goal and robot in x-direction
        u_gtg.x = goalpos.x - currentpos.x;
        // distance between goal and robot in y-direction
        u_gtg.y = goalpos.y - currentpos.y;

        //normalization
        u_gtg_n.x = u_gtg.x/ (sqrt(pow(u_gtg.x, 2)+pow(u_gtg.y, 2)));
        u_gtg_n.y = u_gtg.y/ (sqrt(pow(u_gtg.x, 2)+pow(u_gtg.y, 2)));

        for (int i=0; i<5; i++)
        {   
            u_ao.x += (obstacle_pos[i][0] - currentpos.x) * lasergain[i];
            u_ao.y += (obstacle_pos[i][1] - currentpos.y) * lasergain[i];
        }

        //normalization
        u_ao_n.x = u_ao.x/ (sqrt(pow(u_ao.x, 2)+pow(u_ao.y, 2)));
        u_ao_n.y = u_ao.y/ (sqrt(pow(u_ao.x, 2)+pow(u_ao.y, 2)));

        //blending vector
        u_ao_gtg.x = u_ao_n.x*alpha + u_gtg_n.x*(1-alpha);
        u_ao_gtg.y = u_ao_n.y*alpha + u_gtg_n.y*(1-alpha);

        u_ao_gtg.theta = atan2(u_ao_gtg.y, u_ao_gtg.x);
        
        double e_theta = u_ao_gtg.theta - currentpos.theta;
        // cout << currentpos.theta-theta_ao << endl;

        // cout << "Theta: " << theta_ao << ", " << currentpos.theta << " \n";
        
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

void GTG_AO_Sim::stopCallback(const nav_msgs::Odometry& odom) 
{   
    currentpos.x = odom.pose.pose.position.x;
    currentpos.y = odom.pose.pose.position.y;
    double theta = quatoeuler_yaw(odom);
    cout << "Xc pose: " << currentpos.x << " Xg pose: " << goalpos.x << " Yc pose: " << currentpos.y << " Yg pose: "<< goalpos.y <<endl;
    if ( ( abs(currentpos.x - goalpos.x)<0.1 ) && ( abs(currentpos.y - goalpos.y)<0.1 ) && (currentpos.x * goalpos.x > 0.01))
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
    ros::init(argc, argv, "turtlebot_GTG_AO_Sim"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    GTG_AO_Sim gtg_ao_sim(&nh);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("Initializing...");
    ros::spin();
    return 0;
} 