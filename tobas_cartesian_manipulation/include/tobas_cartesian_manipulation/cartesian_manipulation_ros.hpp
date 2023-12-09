#pragma once

#include <sensor_msgs/JointState.h>

#include <dh_kdl/treetaskspacepid.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_tools/jointstate_jntarray_converter.hpp>
#include <tobas_msgs/Odometry.h>
#include <tobas_msgs/CartesianState.h>

namespace tobas_cartesian_manipulation
{
class CartesianManipulationRos : public tobas::BaseNode
{
  using self = CartesianManipulationRos;
  using super = tobas::BaseNode;

public:
  explicit CartesianManipulationRos(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  tobas::Drone drone_;
  KDL::TreeJointParser jnt_parser_;
  tobas::JointStateJntArrayConverter js_converter_;

  size_t nj_;
  KDL::JntArray jntarraynull_;
  
  tobas_msgs::OdometryConstPtr odom_;
  sensor_msgs::JointStateConstPtr js_;
  tobas_msgs::CartesianStateConstPtr cs_;
  bool is_initialized_ = false;
  KDL::Frame tar_pi_;
  KDL::Frame T_W_B_;

  // Publishers
  ros::Publisher js_cmd_pub_;

  // Subscribers
  ros::Subscriber odom_sub_;
  ros::Subscriber js_sub_;
  ros::Subscriber cs_sub_;

  // Timer
  ros::Timer check_topics_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void odomCb(const tobas_msgs::OdometryConstPtr& odom);
  void jointStateCb(const sensor_msgs::JointStateConstPtr& js);
  void cartStateCb(const tobas_msgs::CartesianStateConstPtr& cs);

  void checkTopicsTimerCb(const ros::TimerEvent& e);
};
}  // namespace tobas_cartesian_manipulation
