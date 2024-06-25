#pragma once

#include <ros/ros.h>

/**
 * @brief ROS_FATALの後にノードを落とす．
 * 例外を吐かずにプログラムを落とすためにNodeHandleを引数にとっている．
 */
#define ROS_EXIT(nh, msg)                                                                                              \
  {                                                                                                                    \
    ROS_FATAL_STREAM(msg);                                                                                             \
    nh.shutdown();                                                                                                     \
  }

/**
 * @copybrief ROS_EXIT
 */
#define ROS_EXIT_NAMED(nh, name, msg) ROS_EXIT(nh, "[" << name << "] " << msg)

/**
 * @brief デバッグモードでも機能するアサーション．
 * Falseの場合は例外を吐くのではなくROS_FATALの後にノードを落とす．
 */
#define ROS_CHECK(nh, expr, msg)                                                                                       \
  {                                                                                                                    \
    if (!static_cast<bool>(expr))                                                                                      \
    {                                                                                                                  \
      ROS_FATAL_STREAM(msg);                                                                                           \
      nh.shutdown();                                                                                                   \
    }                                                                                                                  \
  }
