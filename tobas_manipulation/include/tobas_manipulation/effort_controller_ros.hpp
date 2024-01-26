#pragma once

#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/JointState.h>

#include <tobas_kdl/treejointstateconverter.hpp>
#include <tobas_kdl/treeactivejointsextractor.hpp>
#include <tobas_kdl/treejntspacepid.hpp>
#include <tobas_kdl/treetaskspacepid.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_msgs/Odometry.h>
#include <tobas_msgs/JointEfforts.h>
#include <tobas_msgs/CartesianState.h>

#include <tobas_manipulation/EffortControllerConfig.h>

namespace tobas_manipulation
{
class EffortControllerRos : public tobas::BaseNode
{
  using self = EffortControllerRos;
  using super = tobas::BaseNode;

  using ConfigType = tobas_manipulation::EffortControllerConfig;
  using ConfigServer = dynamic_reconfigure::Server<ConfigType>;

public:
  explicit EffortControllerRos(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  tobas::Drone drone_;
  KDL::TreeJointStateConverter cur_js_conv_;
  KDL::TreeJointStateConverter tar_js_conv_;
  KDL::TreeActiveJointsExtractor active_jnts_extractor_;
  KDL::TreeJntSpacePID pid_js_;
  KDL::TreeTaskSpacePID pid_ts_;

  KDL::JntArray jntarraynull_;

  tobas_msgs::OdometryConstPtr odom_;
  sensor_msgs::JointStateConstPtr cur_js_;
  sensor_msgs::JointStateConstPtr tar_js_;
  tobas_msgs::CartesianStateConstPtr tar_cs_;

  // Publishers
  ros::Publisher efforts_pub_;

  // Subscribers
  ros::Subscriber odom_sub_;
  ros::Subscriber cur_js_sub_;
  ros::Subscriber tar_js_sub_;
  ros::Subscriber tar_cs_sub_;

  // Dynamic Reconfigure Server
  ConfigServer server_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void setInitTargetJointStates();
  int jointSpaceControl(tobas_msgs::JointEfforts& efforts_msg);
  int taskSpaceControl(tobas_msgs::JointEfforts& efforts_msg);

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void odomCb(const tobas_msgs::OdometryConstPtr& odom);
  void currentJointStateCb(const sensor_msgs::JointStateConstPtr& cur_js);
  void targetJointStateCb(const sensor_msgs::JointStateConstPtr& tar_js);
  void targetCartStateCb(const tobas_msgs::CartesianStateConstPtr& tar_cs);

  void dynamicReconfigureCb(const ConfigType& cfg, size_t);
};
}  // namespace tobas_manipulation
