# Turtlebot_PID_Controller 
## Maintainer
- [Hanqing Zhang], <<hzhang8@wpi.edu>>, WPI   

## Description
This Project include:

1. Go to angle controller
2. Go to goal controller
3. Trajectory tracking controller
4. Path generator
5. Avoid obstacle controller
6. Go to goal while avoiding obstacle controller
  
## Read this first
- Set up Turtlebot2 ROS package
- Set up Gazebo 7

## Dependencies

- ROS Kinetic
- Ubuntu 16.04
- Gazebo 7.0.0
- Python 2
- Turtlebot 2

## Run
1. Test Turtlebot ROS package and Gazebo setup
```
roslaunch turtlebot_gazebo turtlebot_world.launch
roslaunch turtlebot_rviz_launchers view_robot.launch
roslaunch turtlebot_teleop keyboard_teleop.launch
```
Note: If everything works, you could see the following environment, and you could also control the robot to move around:   
![Gazebo](https://github.com/Zhanghq8/Turtlebot_PID_Controller/blob/master/Result/Gazebo.png)   
![Rviz](https://github.com/Zhanghq8/Turtlebot_PID_Controller/blob/master/Result/Rviz.png)   
![Keyboard_teleoperator](https://github.com/Zhanghq8/Turtlebot_PID_Controller/blob/master/Result/Keyboard_teleoperator.png)

## Demo
- Gotogoal controller demo(https://youtu.be/xyt6erl5iEk)
- Avoid obstacle controller demo(https://youtu.be/bHeEsdVzJBI) 
- Gotogoal while avoid obstacle controller demo(https://youtu.be/G8c3fivk26k)  