#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs/FluidPressure.h>

#include "../include/tobas_navio_ros/barometer_handler.hpp"
#include "../include/tobas_navio_ros/common.hpp"

using namespace std;

namespace tobas_navio_ros
{
BarometerHandler::BarometerHandler(rclcpp::Node::SharedPtr node, rclcpp::Node::SharedPtr pnh, const string& name) : super(node, pnh, name)
{
  initialize();

  bar_pub_ = nh_.advertise<tobas_hal_msgs::FluidPressure>(hal::kAirPressureTopic, 1);
  main_timer_ = nh_.createTimer(kSamplingRate, &self::mainTimerCb, this);
}

void BarometerHandler::initialize()
{
  if (!barometer_.initialize())
    TOBAS_EXIT("Failed to initialize barometer.");

  if (!barometer_.update())
    TOBAS_EXIT("Failed to update barometer.");
}

void BarometerHandler::mainTimerCb(const rclcpp::TimerEvent& event)
{
  // バロメータを更新
  if (!barometer_.update())
  {
    TOBAS_ERROR_THROTTLE(kErrorPeriod, "Failed to update barometer.");
    return;
  }

  // メッセージを作成
  const auto bar_msg = boost::make_shared<tobas_hal_msgs::FluidPressure>();
  bar_msg->header.stamp = event.current_real;
  bar_msg->fluid_pressure = barometer_.getPressure();

  // メッセージを発行
  bar_pub_.publish(bar_msg);
}
}  // namespace tobas_navio_ros
