#pragma once

#include <ros/ros.h>

#include <dh_kdl/treefksolverpos.hpp>
#include <dh_kdl/treejnttoinertiasolver.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_tools/rotor_axis_extractor.hpp>
#include <tobas_msgs/PoseTwist.h>
#include <tobas_msgs/RotorSpeeds.h>

namespace tobas_mr_wind_estimation
{
class WindEstimator : public tobas::BaseNode
{
  using super = tobas::BaseNode;

public:
  explicit WindEstimator(
    ros::NodeHandle nh,
    ros::NodeHandle pnh,
    const std::string& name = ros::this_node::getName());

  void updateInternalDataStructures();

private:
  tobas::Drone drone_;

  KDL::TreeFkSolverPos fk_solver_;
  KDL::TreeJntToInertiaSolver inertia_solver_;
  tobas::RotorAxisExtractor z_rotors_;
  double mass_;

  tobas_msgs::RotorSpeedsConstPtr rotor_speeds_;

  // PubSub
  ros::Publisher wind_pub_;
  ros::Subscriber pt_sub_;
  ros::Subscriber rotor_speeds_sub_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  Eigen::Matrix3d velCoef(const KDL::Euler& R_W_B);

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void poseTwistCb(const tobas_msgs::PoseTwistConstPtr& pt);
  void rotorSpeedsCb(const tobas_msgs::RotorSpeedsConstPtr& rotor_speeds);
};
}  // namespace tobas_mr_wind_estimation
