#pragma once

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/timer.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/BaseState.h>
#include <tobas_msgs/VelocityYaw.h>
#include <tobas_msgs/LandAction.h>

namespace tobas_state_checker
{
class MultirotorStateChecker : public tobas::BaseNode
{
  static constexpr char kLandActionName[] = "multirotor_landing";
  static constexpr double kSleepTime = 0.1;           // [s]
  static constexpr double kWaitForActionServer = 3.;  // [s]
  static constexpr double kBaseStateTimeout = 0.5;    // [s]
  static constexpr double kCommandTimeout = 0.5;      // [s]
  static constexpr double kAttitudeThreshold = dh_std::deg2rad(80.);

  using super = tobas::BaseNode;

public:
  explicit MultirotorStateChecker();

  void run();

private:
  tobas_msgs::BaseState bs_;
  bool bs_received_;
  bool cmd_received_;
  ros::Time t_last_bs_;
  ros::Time t_last_cmd_;
  tobas_msgs::Event event_;

  ros::Publisher event_pub_;
  ros::Subscriber bs_sub_;
  ros::Subscriber cmd_sub_;

  actionlib::SimpleActionClient<tobas_msgs::LandAction> ac_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void requestShutdown();

  void eventCb(const tobas_msgs::Event& event) override;
  void baseStateCb(const tobas_msgs::BaseState& bs);
  void commandCb(const tobas_msgs::VelocityYaw& cmd);
};
}  // namespace tobas_state_checker
