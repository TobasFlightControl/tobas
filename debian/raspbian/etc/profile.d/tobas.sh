#!/bin/sh

export ROS_DOMAIN_ID=20
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# The Tobas Bash setup must only be sourced by Bash login shells.
if [ -n "${BASH_VERSION:-}" ]; then
  source /opt/tobas/setup.bash
fi
