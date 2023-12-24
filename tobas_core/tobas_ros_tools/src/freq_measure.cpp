#include "../include/tobas_ros_tools/freq_measure.hpp"
#include "../include/tobas_ros_tools/console_message.hpp"

using namespace std;

namespace tobas_ros
{
FreqMeasure::FreqMeasure(const string& name, const double& warn_period, const double& warn_rate)
  : name_(name), warn_period_(warn_period), warn_rate_(warn_rate), ready_(false), first_sleep_(true)
{
  assert(warn_period > 0.);
  assert(0. <= warn_rate && warn_rate <= 1.);
}

void FreqMeasure::setFreq(const double& freq)
{
  assert(freq > 0.);

  freq_ = freq;
  cnt_ideal_ = freq * warn_period_;
  cnt_th_ = cnt_ideal_ * warn_rate_;

  ready_ = true;
  first_sleep_ = true;
}

void FreqMeasure::count()
{
  assert(ready_);

  // 最初のループでは警告を行わず値をリセットするだけ
  if (first_sleep_)
  {
    first_sleep_ = false;
    cnt_ = 0;
    t_last_loop_ = ros::Time::now();
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
      ROS_WARN_STREAM(name_ << ": " << freq << "Hz < " << freq_ << "Hz");
    }
    cnt_ = 0;
    t_last_loop_ = now;
  }
}
}  // namespace tobas_ros
