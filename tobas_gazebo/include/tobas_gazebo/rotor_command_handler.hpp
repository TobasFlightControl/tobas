#pragma once

#include <tobas_tools/node.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_msgs/Battery.h>
#include <tobas_msgs/RotorSpeeds.h>

namespace tobas_gazebo
{
/**
 * @brief ロータ回転数のコマンドを受け取り，スロットルに変換してGazeboの各モータに指令する．
 */
class RotorCommandHandler : public tobas::BaseNode
{
  using self = RotorCommandHandler;
  using super = tobas::BaseNode;

public:
  explicit RotorCommandHandler(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  tobas::Drone drone_;
  tobas_msgs::BatteryConstPtr battery_;

  ros::Publisher throttles_pub_;
  ros::Subscriber battery_sub_;
  ros::Subscriber tar_speeds_sub_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void batteryCb(const tobas_msgs::BatteryConstPtr& battery);
  void targetRotorSpeedsCb(const tobas_msgs::RotorSpeedsConstPtr& tar_speeds);
};
}  // namespace tobas_gazebo
