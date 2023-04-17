#pragma once

#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include <geometry_msgs/Vector3.h>

#include <dh_kdl/treejnttoinertiasolver.hpp>

#include <multirotor_tools/rotor_property.hpp>

class AccelerationController
{
public:
  AccelerationController(const KDL::Tree& tree);

  void updateInternalDataStructures();

  void update(
    const geometry_msgs::Vector3& acc_des,
    const double& yaw,
    double& U_out,
    double& roll_out,
    double& pitch_out);

private:
  const double gravity_;
  const double battery_voltage_;
  const RotorConfigs rotor_configs_;
  double max_U_;

  KDL::TreeJntToInertiaSolver inertia_solver_;
  double mass_;
};
