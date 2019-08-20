#!/usr/bin/env bash

source /opt/ros/kinetic/setup.bash
. ~/catkin_ws/devel/setup.bash
export ROSCONSOLE_FORMAT='[${severity}] - ${node}: [${time}] ${message}'

roslaunch connector connector.launch

sleep 600