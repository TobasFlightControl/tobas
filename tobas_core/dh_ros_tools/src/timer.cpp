#include "../include/dh_ros_tools/timer.hpp"

#define SLEEP 0.1

namespace dh_ros
{
Timer::~Timer()
{
  timer_.stop();
}

void Timer::start()
{
  // spin & sleepの後にタイマーを起動することで，同時刻に複数回呼ばれることを防ぐ
  // ros::spinOnce();  // NodeletのonInit()でspin()を呼ぶとバグる
  // ros::Duration(SLEEP).sleep();

  timer_.start();
}

void Timer::stop()
{
  timer_.stop();
}

Timer& Timer::operator=(const ros::Timer& other)
{
  timer_ = other;
  return *this;
}
}  // namespace dh_ros
