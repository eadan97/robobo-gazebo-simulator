# robobo-gazebo-simulator

Repository for Robobo robot simulation in the Gazebo environment.

## Requirements

* Ubuntu 16.04
* Robot Operating System - Kinetic
* Gazebo7

## Installation

This model uses the original Robobo ROS messages, so it is necessary to use the robobo_msgs package, avaliable on https://github.com/mintforpeople/robobo-ros-msgs/tree/master/robobo_msgs.
Clone the respository in your own workspace and compile:


```bash
$ cd <catkin_ws>/src
$ git clone https://github.com/mintforpeople/robobo-gazebo-simulator
$ git clone https://github.com/mintforpeople/robobo-ros-msgs
$ catkin_make
```

## Basic Usage

You must do the first two steps in each new terminal you need to use the model:

```bash
$ cd <catkin_ws>/src
$ source devel/setup.bash
```

To launch the model:

```bash
$ roslaunch robobo_gazebo robobo.launch
```

To interact with the model you have the following ROS topics and services. They are the same ones used in the real Robobo, there is more information here: https://github.com/mintforpeople/robobo-programming/wiki/ROS.

Topics availables:

* /\<modelName\>/robot/moveWheels
* /\<modelName\>/robot/resetWheels
* /\<modelName\>/robot/movePanTilt
* /\<modelName\>/robot/unlock/move
* /\<modelName\>/robot/accel
* /\<modelName\>/robot/camera/camera_info
* /\<modelName\>/robot/camera/image/compressed
* /\<modelName\>/robot/irs
* /\<modelName\>/robot/orientation
* /\<modelName\>/robot/pan
* /\<modelName\>/robot/tilt
* /\<modelName\>/robot/wheels

Services availables:

* /\<modelName>\/moveWheels
* /\<modelName>\/resetWheels
* /\<modelName>\/movePanTilt

\<modelName\> is robot by default but it can be changed for other name in the launch file.

## Multi robot enviroment

This Gazebo package supports multi robot enviroments. To spawn a Robobo you should add a group tag like the following to the launch file (see [here](launch/robobo.launch)):

```xml
<group ns="NAME">
     <param name="tf_prefix" value="NAME_tf" />
     <arg name="robobo_name" default="NAME"/>
     <node name="robobo_model" pkg="gazebo_ros" type="spawn_model" args="-file $(find robobo_gazebo)/models/robobo/model.sdf -sdf -x 1.0 -y -0.0 -z 0.0 -Y 3.14159 -model $(arg robobo_name)" />
     <node name="robobo_irs" pkg="robobo_gazebo" type="robobo_irs" args="-n $(arg robobo_name)"/>
</group>
```

Where *NAME* should be changed to a unique name that will help to identify that specific robot. So each time you want to execute a script that communicates with that Robobo remember to specify the namespace; ROS allows you to change that using the private variable "*_ns*".

### Example

> rosrun robobo_gazebo moveTest.py __ns:=robobo1 _lspeed:=60 _time:=1000

The previous command will run the moveTest.py script and will set the private variables *lspeed*, *time* and *_ns*; effectively commanding the robobo1 to move left wheel at 60 speed for 1000 milliseconds.

## Structure

This package contains the following folders:

* include: contains header files for the source code.
* launch: contains the launch files used by roslaunch.
* models: contains the models used by Gazebo.
* nodes: contains the node used to republish IR sensons data.
* plugins: contains the source code of the plugins used by the *robobo* model.
* scripts: contains test scripts and a validation script.
* src: contains the source code used by the IR node.
* world: contains a definition of the world used by Gazebo.

## Remark

This package includes one node in python with the function of publishing all infrared sensor values in only one topic. This program reads all topics published by plugin infrared_range.cpp in each ray sensor of the model and brings them all together in one topic, like in the real Robobo.

## License

robobo-gazebo-simulator is available under the Apache 2.0 license. See the LICENSE file for more info.

## Acknowledgement
<!-- 
    ROSIN acknowledgement from the ROSIN press kit
    @ https://github.com/rosin-project/press_kit
-->

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png" 
       alt="rosin_logo" height="60" >
</a>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.  
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg" 
     alt="eu_flag" height="45" align="left" >  

This project has received funding from the European Union’s Horizon 2020  
research and innovation programme under grant agreement no. 732287. 



