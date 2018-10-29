[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

# ROS Publisher and Subscriber Assignment

## Description

This repository will cover the installation of ROS with basic implementation of 
publisher and sbuscriber nodes where we'll have a publisher node(/talker) and a 
subscriber node(/listner) and the transfer of data(string messages) will be over
a topic(/chatter).

## Overview

One package is built(beginner_tutorials) which has two nodes which are defined
in a C++ source code. The Publisher node publishes a string message on the topic 
and the subscriber node reads the string message from that topic. Following is
the graphical representation.

<p align="center">
<img src="image/rosgraph.png" width="70%" height="70%"> 
</p>

## Dependencies

- Ubuntu 16.04
- ROS Kinetic
- python (if not already installed)
- catkin_pkg (if not already installed while installing ROS)
For installing catkin_pkg and directly adding it to PYTHONPATH.
```
pip install catkin_pkg
```
Check if the catkin_pkg path is added to PYTHONPATH by using the following
command
```
echo $PYTHONPATH
```

## Installation

ROS Installation:
Install the ROS Kinetic for Ubuntu and it's dependencies using the [link](http://wiki.ros.org/kinetic/Installation/Ubuntu)


## Build Instructions

Run the following commands for building and running the Publisher and Subscriber
Nodes:

To make the catkin workspace:
```
sudo rm -R ~/catkin_ws (skip this step if no such folder is found)
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone --recursive https://github.com/KrishnaBhatu/beginner_tutorials.git
cd ..
catkin_make
```

Now the package is ready to use:
We have to open up 3 terminals to run the master, publisher and subscriber nodes.

In first terminal
```
source /opt/ros/kinetic/setup.bash
roscore
```
In second terminal
```
cd ~/catkin_ws
source devel/setup.bash
rosrun beginner_tutorials beginner_tutorialsPublisher
```

In third terminal
```
cd ~/catkin_ws
source devel/setup.bash
rosrun beginner_tutorials beginner_tutorialsSubscriber
```
Thus we will see the string messages which are published on the /chatter topic
in the third terminal. 

