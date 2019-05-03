#include "turtlebot_pid/path_generator.h"

//get the goal pos 

//calculate the heading angle

//get current pos of the turtlebot

//stop turtlebot when the goal is reached


Path_Generator::Path_Generator(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("In class constructor of Path_Generator");
    initPub();
    setcircularpath();
    setcircularcenter();
    path_pub();

}

void Path_Generator::setcircularpath(double r, double re)
{
    radius = r;
    resolution = re;
}

void Path_Generator::setcircularcenter(double x, double y)
{
    x_center = x;
    y_center = y;
}


// void Path_Generator::initSub()
// {
//     ROS_INFO("Initializing Subscribers");  
//     currentpos_sub_ = nh_.subscribe("/odom", 1, &Path_Generator::currentposCallback,this);
//     goalpos_sub_ = nh_.subscribe("/turtlebot/path", 1, &Path_Generator::goalposCallback,this); 
//     // stop_sub_ = nh_.subscribe("/odom", 1, &Path_Generator::stopCallback,this);
// }


//member helper function to set up publishers;
void Path_Generator::initPub()
{
    ROS_INFO("Initializing Publishers");
    path_pub_ = nh_.advertise<geometry_msgs::Pose2D>("/turtlebot/path", 1, true); 
}


void Path_Generator::path_pub() 
{   
    // ros::Rate loop_rate(10);
    while (ros::ok())
    {
        angle += PI *resolution/180;
        pathpos.x = x_center + radius*cos(angle);
        pathpos.y = y_center + radius*sin(angle);
        path_pub_.publish(pathpos); //output the square of the received value; 
        cout << "goal X: " << pathpos.x << ". goal Y: " << pathpos.y << "." << endl;
        if (abs(angle - 2*PI) <= 0.0001)
        {
            angle = 0;
        }
        ros::Duration(0.1).sleep();
    }

}


int main(int argc, char** argv) 
{
    ros::init(argc, argv, "turtlebot_path_generator"); //node name
    ROS_INFO("Initializing...");
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    Path_Generator path_generator(&nh);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

    // ros::spin();
    return 0;
} 

