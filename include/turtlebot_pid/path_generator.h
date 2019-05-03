#ifndef PATH_GENERATOR_H_
#define PATH_GENERATOR_H_

#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include <ros/ros.h>

#include <geometry_msgs/Pose2D.h>

// https://answers.ros.org/question/31779/how-to-use-a-message-that-i-made/

const double PI = 3.14159;

using namespace std;

class Path_Generator
{
public:
    Path_Generator(ros::NodeHandle* nodehandle);

private:

    double radius;
    double resolution;
    double angle = 0.0;
    double x_center;
    double y_center;

    geometry_msgs::Pose2D pathpos;

    ros::NodeHandle nh_;

    ros::Publisher path_pub_;
 
    void initPub();

    void setcircularpath(double r=2.0, double re=0.2);
    void setcircularcenter(double x=3.0, double y=2.0);

    void path_pub(); 
};

#endif

    // https://github.com/wsnewman/ros_class/blob/master/example_ros_class/src/example_ros_class.h