# ROS Tutorials - Publisher and Subscriber


## Overview

This repository depicts the basic functionality of publisher and subscriber nodes os ROS.It is based on the tutorials from http://wiki.ros.org/ROS/Tutorials/ . In this tutorial a string is given as output when publisher and subscriber nodes are called.This is created depict the usage of service file and logging levels of ros.

## Dependencies
1) ROS distro: 'Kinetic'. 
2) Catkin installed

## Standard build and run via command-line
```
cd <path to catkin workspace>/src
git clone --recursive https://github.com/vamshibogoju/beginner_tutorials.git
cd ../..
catkin_make
```
In new terminal:
```
roscore
```
In another terminal:
```
cd ~/catkin_ws
source ./devel/setup.bash
rosrun beginner_tutorials talker
```

In another terminal:
```
cd ~/catkin_ws
source ./devel/setup.bash
rosrun beginner_tutorials listener
```
## Launch File
To use launch file type the following command in the terminal:
```
roslaunch beginner_tutorials week10hw.launch
```
User can change the frequency at which the loop operates by the following command;
```
roslaunch begineer_tutorials week10HW.launch frequency:=5

# Service

If the user would like to change the output string message, type the following command in a new terminal
```
rosservice call /changeText "sample text"
```

# Logging in RQT console
To see the message log in real time, use rqt_console GUI by typing the following command in a new terminal: 
```
rqt_console
```
## TF frames

The talker node has been changed to boradcast static tf frames.To visualize the frames, use the following commands in the terminal seperately

```
rosrun rqt_tf_tree rqt_tf_tree
rosrun tf tf_echo /world /talk
rosrun tf view_frames
```

## Unit testing

For testing the program, use the launch file in the test folder. 
Run the following commands:
```
cd ~/catkin_ws
catkin_make
rostest beginner_tutorials talkerTest.launch
```

Following ouput is generated:
```
[Testcase: testtalkerTest] ... ok

[ROSTEST]-----------------------------------------------------------------------

[beginner_tutorials.rosunit-talkerTest/testServiceExsistance][passed]
[beginner_tutorials.rosunit-talkerTest/testServiceMessageUpdate][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 0

rostest log file is in /home/vamshi/.ros/log/rostest-vamshi-G5-5587-6651.log
```

## Rosbag usage

To run the ros bag, use the following command to start recording for 10 seconds: (Change the value to the desired amount of time in this command)
roslaunch the launch file then run below command 

```
rosbag record --duration=10 -a -O record.bag
```
This records for 10 seconds and saves it as a rosbag file in the directory where terminal is working. To play the rosbag file saved in my repo, run the following commands in new terminals:
```
cd ~/catkin_ws
rosrun beginner_tutorials listener
```
In another terminal, type:
```
cd ~/catkin_ws/src/beginner_tutorials/results
rosbag play record.bag

## Cpplint check
Execute the following commands in a new terminal to run cpplint
```
cd  <path to repository>
cpplint $( find . -name \*.hpp -or -name \*.cpp )
```

## Cppcheck check
Execute the following commands in a new terminal to run cppcheck
```
cd <path to repository>
cppcheck --enable=all --std=c++11 -I include/ --suppress=missingIncludeSystem $( find . -name *.cpp )
```
