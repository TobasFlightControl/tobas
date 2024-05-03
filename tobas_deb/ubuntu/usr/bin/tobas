#!/bin/bash

export ROS_IP=`hostname -I | cut -d' ' -f1`
export ROS_HOSTNAME=`hostname -I | cut -d' ' -f1`
export ROS_MASTER_URI=http://localhost:11311

source /opt/ros/noetic/setup.bash
source /opt/tobas/setup.bash

roslaunch tobas_gcs gcs.launch
