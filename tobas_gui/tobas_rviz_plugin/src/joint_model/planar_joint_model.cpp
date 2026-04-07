// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/joint_model/planar_joint_model.hpp"

#include <cmath>
#include <limits>

#include <angles/angles.h>
#include <geometric_shapes/check_isometry.h>

#include <tobas_math/definitions.hpp>

namespace tobas
{
PlanarJointModel::PlanarJointModel(const std::string& name, size_t joint_index, size_t first_variable_index)
  : JointModel(name, joint_index, first_variable_index)
  , angular_distance_weight_(1.)
  , motion_model_(HOLONOMIC)
  , min_translational_distance_(1e-5)
{
  type_ = PLANAR;

  local_variable_names_.push_back("x");
  local_variable_names_.push_back("y");
  local_variable_names_.push_back("theta");
  for (int i = 0; i < 3; ++i) {
    variable_names_.push_back(getName() + "/" + local_variable_names_[i]);
    variable_index_map_[variable_names_.back()] = i;
  }

  variable_bounds_.resize(3);
  variable_bounds_[0].position_bounded_ = true;
  variable_bounds_[1].position_bounded_ = true;
  variable_bounds_[2].position_bounded_ = false;

  variable_bounds_[0].min_position_ = -std::numeric_limits<double>::infinity();
  variable_bounds_[0].max_position_ = std::numeric_limits<double>::infinity();
  variable_bounds_[1].min_position_ = -std::numeric_limits<double>::infinity();
  variable_bounds_[1].max_position_ = std::numeric_limits<double>::infinity();
  variable_bounds_[2].min_position_ = -M_PI;
  variable_bounds_[2].max_position_ = M_PI;

  computeVariableBoundsMsg();
}

uint32_t PlanarJointModel::getStateSpaceDimension() const
{
  return 3;
}

void PlanarJointModel::getVariableDefaultPositions(double* values, const Bounds& bounds) const
{
  for (uint32_t i = 0; i < 2; ++i) {
    // if zero is a valid value
    if (bounds[i].min_position_ <= 0. && bounds[i].max_position_ >= 0.) {
      values[i] = 0.;
    }
    else {
      values[i] = (bounds[i].min_position_ + bounds[i].max_position_) / 2.;
    }
  }
  values[2] = 0.;
}

void computeTurnDriveTurnGeometry(
  const double* from,
  const double* to,
  const double min_translational_distance,
  double& dx,
  double& dy,
  double& initial_turn,
  double& drive_angle,
  double& final_turn)
{
  dx = to[0] - from[0];
  dy = to[1] - from[1];
  // If the translational distance between from & to states is very small, it will cause an unnecessary rotation since
  // the robot will try to do the following rather than rotating directly to the orientation of `to` state
  // 1- Align itself with the line connecting the origin of both states
  // 2- Move to the origin of `to` state
  // 3- Rotate so it have the same orientation as `to` state
  // Example: from=[0., 0., 0.] - to=[1e-31, 1e-31, -130°]
  // here the robot will: rotate 45° -> move to the origin of `to` state -> rotate -175°, rather than rotating directly
  // to -130°
  // to fix this we added a joint property (default value is 1e-5) and make the movement pure rotation if the
  // translational distance is less than this number
  const double angle_straight_diff = std::hypot(dx, dy) > min_translational_distance ?
                                       angles::shortest_angular_distance(from[2], std::atan2(dy, dx)) :
                                       0.;
  const double angle_backward_diff = angles::normalize_angle(angle_straight_diff - M_PI);
  const double move_straight_cost =
    std::abs(angle_straight_diff) + std::abs(angles::shortest_angular_distance(from[2] + angle_straight_diff, to[2]));
  const double move_backward_cost =
    std::abs(angle_backward_diff) + std::abs(angles::shortest_angular_distance(from[2] + angle_backward_diff, to[2]));
  if (move_straight_cost <= move_backward_cost) {
    initial_turn = angle_straight_diff;
  }
  else {
    initial_turn = angle_backward_diff;
  }
  drive_angle = from[2] + initial_turn;
  final_turn = angles::shortest_angular_distance(drive_angle, to[2]);
}

void PlanarJointModel::computeTransform(const double* joint_values, Eigen::Isometry3d& transform) const
{
  transform = Eigen::Isometry3d(
    Eigen::Translation3d(joint_values[0], joint_values[1], 0.) *
    Eigen::AngleAxisd(joint_values[2], Eigen::Vector3d::UnitZ()));
}

void PlanarJointModel::computeVariablePositions(const Eigen::Isometry3d& transform, double* joint_values) const
{
  joint_values[0] = transform.translation().x();
  joint_values[1] = transform.translation().y();

  ASSERT_ISOMETRY(transform)  // unsanitized input, could contain a non-isometry
  Eigen::Quaterniond q(transform.linear());
  // taken from Bullet
  double s_squared = 1. - (q.w() * q.w());
  if (s_squared < 10. * std::numeric_limits<double>::epsilon()) {
    joint_values[2] = 0.;
  }
  else {
    double s = 1. / sqrt(s_squared);
    joint_values[2] = (acos(q.w()) * 2.0f) * (q.z() * s);
  }
}
}  // namespace tobas
