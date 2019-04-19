# Project 2: Go Chase It
Project 2 of the Udacity Robotics Nanodegree

This is the second project for the Udacity Robotics Nanodegree.


# Overview
THis project consists of 2 ROS packages located inside the ```catkin_ws``` workspace: ```drive_bot```, and ```ball_chaser```.

# To View Project
Prerequisites: ROS and Gazebo installed on Linux workspace

1. Clone the repository into an empty folder ```catkin_ws```
2. Initialise the catkin workspace
```
$ cd catkin_ws/src
$ catkin_init_workspace
```
3. Build files and launch the ```my_robot``` node
```
$ cd catkin_ws
$ catkin_make
```
```
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```
4. Launch the ```ball_chaser``` node in a new terminal
```
$ cd catkin_ws
$ source devel/setup.bash
$ roslaunch ball_chaser ball_chaser.launch
```
5. Subscribe to camera RGB image topic from RViz to visualize the robot's camera images
```
$ cd catkin_ws
$ source devel/setup.bash
$ rosrun rqt_image_view rqt_image_view
```
6. Place the white ball at different positions within the gazebo workspace and watch the robot chase the ball!
