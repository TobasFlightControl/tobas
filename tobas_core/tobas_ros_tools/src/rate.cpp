#include "../include/tobas_ros_tools/rate.hpp"
#include "../include/tobas_ros_tools/console_message.hpp"

using namespace std;

namespace tobas_ros
{
Rate::Rate(const double& freq, const double& warn_period, const double& warn_rate)
  : super(freq),
    freq_(freq),
    warn_period_(warn_period),
    cnt_ideal_(freq * warn_period),
    cnt_th_(cnt_ideal_ * warn_rate),
    first_sleep_(true)
{
  ROS_ASSERT(freq > 0.);
  ROS_ASSERT(warn_period > 0.);
  ROS_ASSERT(warn_rate > 0.);
}

void Rate::sleep()
{
  // 最初のループでは警告を行わず値をリセットするだけ
  if (first_sleep_)
  {
    first_sleep_ = false;
    cnt_ = 0;
    t_last_loop_ = ros::Time::now();
    super::sleep();
    return;
  }

  ++cnt_;
  const ros::Time now = ros::Time::now();
  const double elapsed_time = (now - t_last_loop_).toSec();
  if (elapsed_time > warn_period_)
  {
    if (cnt_ < cnt_th_)
    {
      const double freq = freq_ * cnt_ / cnt_ideal_;
      ROS_WARN_STREAM(freq << "Hz < " << freq_ << "Hz");
    }
    cnt_ = 0;
    t_last_loop_ = now;
  }
  super::sleep();
}
}  // namespace tobas_ros
