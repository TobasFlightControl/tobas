#pragma once

#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/JointState.h>

#include <tobas_kdl/treejointstateconverter.hpp>
#include <tobas_kdl/treeactivejointsextractor.hpp>
#include <tobas_kdl/treetaskspacevelctrl.hpp>
#include <tobas_ros_tools/tf_listener.hpp>
#include <tobas_tools/node.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_msgs/JointVelocities.h>
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
  KDL::TreeActiveJointsExtractor active_jnts_extractor_;
  KDL::TreeTaskSpaceVelCtrl vel_ctrl_;

  tobas_ros::TransformListener tf_listener_;
  KDL::JntArray jntarraynull_;
  double jnt_time_const_;
  sensor_msgs::JointState home_js_;
  ros::Time t_last_cmd_;
  bool is_commanded_ = false;

  sensor_msgs::JointStateConstPtr cur_js_;
  sensor_msgs::JointStateConstPtr tar_js_;
  tobas_msgs::CartesianStateConstPtr tar_cs_;

  // Publishers
  ros::Publisher velocities_pub_;

  // Subscribers
  ros::Subscriber cur_js_sub_;
  ros::Subscriber tar_js_sub_;
  ros::Subscriber tar_cs_sub_;

  // Dynamic Reconfigure Server
  ConfigServer server_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  int jointSpaceControl(tobas_msgs::JointVelocities& velocities_msg);
  int taskSpaceControl(tobas_msgs::JointVelocities& velocities_msg);

  void currentJointStateCb(const sensor_msgs::JointStateConstPtr& cur_js);
  void targetJointStateCb(const sensor_msgs::JointStateConstPtr& tar_js);
  void targetCartStateCb(const tobas_msgs::CartesianStateConstPtr& cs);

  void dynamicReconfigureCb(const ConfigType& cfg, size_t);
};
}  // namespace tobas_manipulation
