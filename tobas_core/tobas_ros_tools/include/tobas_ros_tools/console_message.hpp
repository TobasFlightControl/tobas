#pragma once

#include <ros/ros.h>

#define rosInfo(name, msg) ROS_INFO_STREAM("[" << name << "] " << msg)
#define rosWarn(name, msg) ROS_WARN_STREAM("[" << name << "] " << msg)
#define rosError(name, msg) ROS_ERROR_STREAM("[" << name << "] " << msg)
#define rosFatal(name, msg) ROS_FATAL_STREAM("[" << name << "] " << msg)

#define rosInfoOnce(name, msg) ROS_INFO_STREAM_ONCE("[" << name << "] " << msg)
#define rosWarnOnce(name, msg) ROS_WARN_STREAM_ONCE("[" << name << "] " << msg)
#define rosErrorOnce(name, msg) ROS_ERROR_STREAM_ONCE("[" << name << "] " << msg)
#define rosFatalOnce(name, msg) ROS_FATAL_STREAM_ONCE("[" << name << "] " << msg)

#define rosInfoThrottle(period, name, msg)                                                         \
  ROS_INFO_STREAM_THROTTLE(period, "[" << name << "] " << msg)
#define rosWarnThrottle(period, name, msg)                                                         \
  ROS_WARN_STREAM_THROTTLE(period, "[" << name << "] " << msg)
#define rosErrorThrottle(period, name, msg)                                                        \
  ROS_ERROR_STREAM_THROTTLE(period, "[" << name << "] " << msg)
#define rosFatalThrottle(period, name, msg)                                                        \
  ROS_FATAL_STREAM_THROTTLE(period, "[" << name << "] " << msg)
