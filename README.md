[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

# ROS Beginners Tutorial

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

One of the node has a service to change the string which is published which 
depends on the input given by the user. Also, there is a launch file generated
which acts as a script for firing up all the nodes mentioned in the script at
once which frees us from opening the terminal one by one and input rosrun for
individual nodes.
 
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

--skip command 2(sudo rm -R ~/catkin_ws) if no such folder is found

```

sudo rm -R ~/catkin_ws
source /opt/ros/kinetic/setup.bash 
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
cd src/
git clone -b Week10_HW https://github.com/KrishnaBhatu/beginner_tutorials.git
cd ..
catkin_make
```

Now the package is ready to use

The above steps are same for bothe the methods (with and without launch files)

# To Run without using launch file 
This is for the basic Publisher Subscriber implementation
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

Now we can run the service to modify the string that is published by using the changeString  service.
The following is the implemetation of client server process.
Open up a new terminal for  calling the service.(Press: ctrl+alt+T)

```
cd ~/catkin_ws
source devel/setup.bash
rosservice call changeString "<any new string the user wants to enter>"
```
Now we observe that the published message changes to our new input string message on the listener terminal.
Close all terminals.

# To Run with using launch file
Open up one terminal and follow the following steps:

```
cd ~/catkin_ws
source devel/setup.bash
roslaunch beginner_tutorials changeString.launch frequency:=<integer value for frequency loop>
```
 
 The frequency parameter is optional.
 Now we can see that all the nodes along with the roscore are fired up.
 
The following is the implemetation of client server process.
Now open up new terminal to call the service and follow the folowing instructions.
 ```
cd ~/catkin_ws
source devel/setup.bash
rosservice call changeString "<any new string the user wants to enter>"
```
Now we observe that the published message changes to our new input string message on the listener terminal.
Close all terminals

# Inspection of TF Frames
For this session we have to keep the roslaunch command in the above section running.
The transformation of the talk frame is broadcasted with respect to the world frame which can be 
inspected by running the following commands.

Following commands are run in a new terminal while the roslaunch command is already running in other terminal
```
source /opt/ros/kinetic/setup.bash
rosrun tf tf_echo /world /talk
```

After running this command we can see the static transformation that are broadcasted as follows:
```
At time 1542168024.163
- Translation: [10.000, 10.000, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.707, 0.707]
            in RPY (radian) [0.000, -0.000, 1.570]
            in RPY (degree) [0.000, -0.000, 89.954]
At time 1542168024.496
- Translation: [10.000, 10.000, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.707, 0.707]
            in RPY (radian) [0.000, -0.000, 1.570]
            in RPY (degree) [0.000, -0.000, 89.954]
At time 1542168025.496
```

To visualize the frames we open up a new tab and run the following:
```
rosrun tf view_frames
evince frames.pdf
```
Close all terminals
# To Run ROS Test
For this section no nodes can be running.(close all terinals)
The following commands are used to run the unit test of our ROS package.
```
cd ~/catkin_ws
source devel/setup.bash
catkin_make run_tests_beginner_tutorials
```
The output of the folling command will be like the following
```
[ROSUNIT] Outputting test results to /home/kbhatu/catkin_ws/build/test_results/beginner_tutorials/rostest-test_unittest.xml
[Testcase: testunittest] ... ok

[ROSTEST]-----------------------------------------------------------------------

[beginner_tutorials.rosunit-unittest/testofService][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 1
 * ERRORS: 0
 * FAILURES: 0

rostest log file is in /home/kbhatu/.ros/log/rostest-kbhatu-HP-Pavilion-Notebook-30547.log
-- run_tests.py: verify result "/home/kbhatu/catkin_ws/build/test_results/beginner_tutorials/rostest-test_unittest.xml"
```

#To Run the bag file
The bag file is used to store the log data from the publisher in a .bag file.
We can save log file by running the following commands by which the data published by the talker will be captured in the .bag file.

Open a new terminal
```
$ cd ~/catkin_make
$ source devel/setup.bash
$ roslaunch beginner_tutorials changeString.launch frequency:=<integer value for frequency loop>  run_rosbag:=true
```

Now the .bag file can be used to playback the previous messages that were published by the talker in the following manner:

Open a one terminal
```
roscore
```
Open another terminal(We will see our output in this terminal)
```
cd ~/catkin_ws
source devel/setup.bash
rosrun beginner_tutorials beginner_tutorialsSubscriber
```
Now we open another terminal and play the log file which will be listened by the listener node.
```
cd <path to repository>/results
rosbag play listenerRosBag.bag
```


