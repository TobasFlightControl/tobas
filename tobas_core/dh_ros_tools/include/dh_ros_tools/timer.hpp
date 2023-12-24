#pragma once

#include <ros/ros.h>
#include <ros/timer.h>

namespace dh_ros
{
/**
 * @brief ros::Timerのラッパー．
 * タイマー作成時にコールバックが複数回呼ばれることを防ぐ．
 * デストラクタでタイマーの停止を行うため，手動で停止しなくても大丈夫．
 */
class Timer
{
  // ros::Timerの子クラスとして実装するとコールバックが呼ばれなかった

public:
  template <typename T>
  explicit Timer(
    ros::NodeHandle& nh,
    const double& period,
    void (T::*callback)(const ros::TimerEvent&),
    T* obj,
    bool auto_start = true);
  ~Timer();

  void start();
  void stop();

  Timer& operator=(const ros::Timer& other);

private:
  ros::Timer timer_;
};

template <typename T>
Timer::Timer(
  ros::NodeHandle& nh,
  const double& period,
  void (T::*callback)(const ros::TimerEvent&),
  T* obj,
  bool auto_start)
{
  ROS_ASSERT(period > 0.);

  timer_ = nh.createTimer(ros::Duration(period), callback, obj, false, false);

  if (auto_start)
  {
    start();
  }
}
}  // namespace dh_ros
