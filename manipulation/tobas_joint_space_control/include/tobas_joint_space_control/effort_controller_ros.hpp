#pragma once

#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/JointState.h>

#include <dh_kdl/treejntspacepid.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_tools/jointstate_jntarray_converter.hpp>

#include <tobas_joint_space_control/EffortControllerConfig.h>

namespace tobas_joint_space_control
{
class EffortControllerRos : public tobas::BaseNode
{
  using self = EffortControllerRos;
  using super = tobas::BaseNode;

  using ConfigType = tobas_joint_space_control::EffortControllerConfig;
  using ConfigServer = dynamic_reconfigure::Server<ConfigType>;

public:
  explicit EffortControllerRos(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  tobas::Drone drone_;
  tobas::JointStateJntArrayConverter cur_js_conv_;
  tobas::JointStateJntArrayConverter tar_js_conv_;
  KDL::TreeJntSpacePID pid_;
  KDL::JntArray jntarraynull_;

  bool is_initialized_ = false;
  sensor_msgs::JointStateConstPtr cur_js_;
  sensor_msgs::JointStateConstPtr tar_js_;

  // Publishers
  ros::Publisher efforts_pub_;

  // Subscribers
  ros::Subscriber cur_js_sub_;
  ros::Subscriber tar_js_sub_;

  // Timer
  ros::Timer check_topics_timer_;

  // Dynamic Reconfigure Server
  ConfigServer server_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void currentJointStateCb(const sensor_msgs::JointStateConstPtr& cur_js);
  void targetJointStateCb(const sensor_msgs::JointStateConstPtr& tar_js);

  void checkTopicsTimerCb(const ros::TimerEvent& e);
  void dynamicReconfigureCb(const ConfigType& cfg, size_t);
};
}  // namespace tobas_joint_space_control
