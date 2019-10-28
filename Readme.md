# ROS Tutorials - Publisher and Subscriber


## Overview

This repository depicts the basic functionality of publisher and subscriber nodes os ROS.It is based on the tutorials from http://wiki.ros.org/ROS/Tutorials/ . In this tutorial a string is given as output when publisher and subscriber nodes are called.

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
