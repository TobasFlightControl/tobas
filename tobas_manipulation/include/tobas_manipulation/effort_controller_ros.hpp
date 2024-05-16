#pragma once

#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/JointState.h>

#include <tobas_kdl/treejointstateconverter.hpp>
#include <tobas_kdl/treeactivejointsextractor.hpp>
#include <tobas_kdl/treejntspacepid.hpp>
#include <tobas_kdl/treetaskspacepid.hpp>
#include <tobas_ros_tools/tf_listener.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_msgs/JointCommandArray.h>
#include <tobas_msgs/LinkStateArray.h>

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
  tobas_kdl::TreeJointStateConverter cur_js_conv_;
  tobas_kdl::TreeJointStateConverter tar_js_conv_;
  tobas_kdl::TreeActiveJointsExtractor active_jnts_extractor_;
  tobas_kdl::TreeJntSpacePID pid_js_;
  tobas_kdl::TreeTaskSpacePID pid_ts_;

  tobas_ros::TransformListener tf_listener_;
  sensor_msgs::JointState home_js_;
  ros::Time t_last_cmd_;
  bool is_commanded_ = false;

  sensor_msgs::JointStateConstPtr cur_js_;
  sensor_msgs::JointStateConstPtr tar_js_;
  tobas_msgs::LinkStateArrayConstPtr tar_ls_;

  // Publishers
  ros::Publisher efforts_pub_;

  // Subscribers
  ros::Subscriber odom_sub_;
  ros::Subscriber cur_js_sub_;
  ros::Subscriber tar_js_sub_;
  ros::Subscriber tar_ls_sub_;

  // Dynamic Reconfigure Server
  ConfigServer server_;

  int jointSpaceControl(tobas_msgs::JointCommandArray& efforts_msg);
  int taskSpaceControl(tobas_msgs::JointCommandArray& efforts_msg);

  void currentJointStateCb(const sensor_msgs::JointStateConstPtr& cur_js);
  void targetJointStateCb(const sensor_msgs::JointStateConstPtr& tar_js);
  void targetLinkStateCb(const tobas_msgs::LinkStateArrayConstPtr& tar_ls);

  void dynamicReconfigureCb(const ConfigType& cfg, size_t);
};
}  // namespace tobas_manipulation
