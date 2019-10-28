# C++ Boilerplate


## Overview

Simple starter ROS project with:

- catkin
- pub+sub node interaction

## Standard install via command-line
```
git clone --recursive https://github.com/rjb3vp/beginner_tutorials.git
cd <path to repository>
mkdir catkin_ws
cd catkin_ws
catkin_make
source ./devel/setup.bash

Run program: 

roscore
rosrun beginner_tutorials listener
rosrun beginner_tutorials talker


```

## Assumptions/Dependencies
```
This project assumes an install of ROS Kinetic on Ubuntu 16.04 LTS with C++11

