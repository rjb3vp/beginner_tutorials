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
cd beginner_tutorials
catkin_make install
(OR just catkin_make if installing into existing catkin workspace)
source ./devel/setup.bash

Run program: 

roscore
rosrun beginner_tutorials listener
rosrun beginner_tutorials talker


OR:


To use the launchfile, run:
roslaunch beginner_tutorials example.launch <optional starting mean argument, as an int>


```

## Services
The random number service allows a client to set the range and mean that talker uses to generate random numbers.
While talker is already running, one can run:

rosservice call /talker/random_data <Mean> <Range>

where mean and range are 64 bit integers.
It will return an error if errors exist.


## Assumptions/Dependencies
```
This project assumes an install of ROS Kinetic on Ubuntu 16.04 LTS with C++11

