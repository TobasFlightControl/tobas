#pragma once

#include <ros/ros.h>

/* A macro that throws an exception. */
#define ROS_THROW(msg)                                                                             \
  {                                                                                                \
    ROS_ERROR_STREAM(msg);                                                                         \
    std::stringstream ss;                                                                          \
    ss << msg;                                                                                     \
    throw ros::Exception(ss.str());                                                                \
  }

/* A macro that throws an exception along with the node name. */
#define ROS_THROW_NAMED(name, msg)                                                                 \
  {                                                                                                \
    ROS_THROW("[" << name << "] " << msg);                                                         \
  }
