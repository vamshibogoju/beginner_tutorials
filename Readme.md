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
roslaunch begineer_tutorials week10hw.launch
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
