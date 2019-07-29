# robobo-gazebo-simulator
Gazebo Simulator for Robobo

Repository for Robobo robot simulation in gazebo environment.

## Requirements

* Ubuntu 16.04
* Robot Operating System - Kinetic
* Gazebo7

## Installation

This model use the original Robobo ROS messages, so it is necessary to use the robobo_msgs package, avaliable on https://github.com/mintforpeople/robobo-ros-msgs/tree/master/robobo_msgs.
Clone the respository in your own workspace and compile:


```bash
$ cd <catkin_ws>/src
$ git clone https://github.com/mintforpeople/robobo-gazebo-simulator
$ git clone https://github.com/mintforpeople/robobo-ros-msgs/tree/master/robobo_msgs
$ catkin_make
```

## Basic Use

You must do the first twos steps in each new terminal you need to use the model:

```bash
$ cd <catkin_ws>/src
$ source devel/setup.bash
```

To launch the model:

```bash
$ roslaunch robobo_gazebo robobo.launch
```

To interact with the model you have the following ROS topics and services. These are the same as used in the real Robobo, there is more information here: https://github.com/mintforpeople/robobo-programming/wiki/ROS.


