#!/bin/bash

# Define error handler
error_handler() {
  echo "Error: $1"
  exit 1
}

# Timeout configurations [s]
TIMEOUT=10
SLEEP_INTERVAL=1
TIME_ELAPSED=0

# Source ROS Noetic
source /opt/ros/noetic/setup.bash || error_handler "Failed to source ROS setup.bash"

# Check ROS_MASTER_URI
if [ -z "$ROS_MASTER_URI" ]; then
  error_handler "ROS_MASTER_URI is not set"
fi

# Check communication with ROS master
until /opt/ros/noetic/bin/rostopic list &>/dev/null; do
  /bin/sleep $SLEEP_INTERVAL
  TIME_ELAPSED=$((TIME_ELAPSED + SLEEP_INTERVAL))
  if [ $TIME_ELAPSED -ge $TIMEOUT ]; then
    error_handler "Timeout while waiting for ROS master"
  fi
done
