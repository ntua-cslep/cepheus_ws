#!/bin/bash
source ./devel/setup.bash
catkin_make
source ./devel/setup.bash
roslaunch cepheus_robot launch_cepheus_robot.launch
