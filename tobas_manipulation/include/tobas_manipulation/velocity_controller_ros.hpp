#pragma once

#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/JointState.h>

#include <dh_kdl/treejointstateconverter.hpp>
#include <dh_kdl/treetaskspacevelctrl.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_msgs/Odometry.h>
#include <tobas_msgs/CartesianState.h>

#include <tobas_manipulation/VelocityControllerConfig.h>

namespace tobas_manipulation
{
class VelocityControllerRos : public tobas::BaseNode
{
  using self = VelocityControllerRos;
  using super = tobas::BaseNode;

  using ConfigType = tobas_manipulation::VelocityControllerConfig;
  using ConfigServer = dynamic_reconfigure::Server<ConfigType>;

public:
  explicit VelocityControllerRos(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  tobas::Drone drone_;
  KDL::TreeJointStateConverter cur_js_conv_;
  KDL::TreeJointStateConverter tar_js_conv_;

  KDL::JntArray jntarraynull_;
  double jnt_time_const_;
  KDL::Vector lin_time_const_, ang_time_const_;

  tobas_msgs::OdometryConstPtr odom_;
  sensor_msgs::JointStateConstPtr cur_js_;
  sensor_msgs::JointStateConstPtr tar_js_;
  tobas_msgs::CartesianStateConstPtr tar_cs_;

  // Publishers
  ros::Publisher velocities_pub_;

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

  int jointSpaceControl(KDL::JntArray& velocities);
  int taskSpaceControl(KDL::JntArray& velocities);

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void odomCb(const tobas_msgs::OdometryConstPtr& odom);
  void currentJointStateCb(const sensor_msgs::JointStateConstPtr& cur_js);
  void targetJointStateCb(const sensor_msgs::JointStateConstPtr& tar_js);
  void targetCartStateCb(const tobas_msgs::CartesianStateConstPtr& cs);

  void dynamicReconfigureCb(const ConfigType& cfg, size_t);
};
}  // namespace tobas_manipulation
