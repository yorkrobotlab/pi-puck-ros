#!/usr/bin/env bash

source /opt/ros/melodic/setup.bash
catkin_make "$@"
source devel/setup.bash
