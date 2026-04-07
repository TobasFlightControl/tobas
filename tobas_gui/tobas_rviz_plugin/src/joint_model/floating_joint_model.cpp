// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/joint_model/floating_joint_model.hpp"

#include <cmath>
#include <limits>

#include <geometric_shapes/check_isometry.h>
#include <eigen3/Eigen/Geometry>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include "tobas_rviz_plugin/logger.hpp"

namespace tobas
{
constexpr size_t STATE_SPACE_DIMENSION = 7;

namespace
{
rclcpp::Logger getLogger()
{
  return tobas::getLogger("tobas.floating_joint_model");
}
}  // namespace

FloatingJointModel::FloatingJointModel(const std::string& name, size_t joint_index, size_t first_variable_index)
  : JointModel(name, joint_index, first_variable_index), angular_distance_weight_(1.)
{
  type_ = FLOATING;
  local_variable_names_.push_back("trans_x");
  local_variable_names_.push_back("trans_y");
  local_variable_names_.push_back("trans_z");
  local_variable_names_.push_back("rot_x");
  local_variable_names_.push_back("rot_y");
  local_variable_names_.push_back("rot_z");
  local_variable_names_.push_back("rot_w");
  for (size_t i = 0; i < STATE_SPACE_DIMENSION; ++i) {
    variable_names_.push_back(getName() + "/" + local_variable_names_[i]);
    variable_index_map_[variable_names_.back()] = i;
  }
}

double FloatingJointModel::distanceTranslation(const double* values1, const double* values2) const
{
  double dx = values1[0] - values2[0];
  double dy = values1[1] - values2[1];
  double dz = values1[2] - values2[2];
  return sqrt(dx * dx + dy * dy + dz * dz);
}

double FloatingJointModel::distanceRotation(const double* values1, const double* values2) const
{
  // The values are in "xyzw" order but Eigen expects "wxyz".
  const auto q1 = Eigen::Quaterniond(values1[6], values1[3], values1[4], values1[5]).normalized();
  const auto q2 = Eigen::Quaterniond(values2[6], values2[3], values2[4], values2[5]).normalized();
  return q2.angularDistance(q1);
}

bool FloatingJointModel::normalizeRotation(double* values) const
{
  // normalize the quaternion if we need to
  double norm_sqr = values[3] * values[3] + values[4] * values[4] + values[5] * values[5] + values[6] * values[6];
  if (std::abs(norm_sqr - 1.) > std::numeric_limits<double>::epsilon() * 100.) {
    double norm = sqrt(norm_sqr);
    if (norm < std::numeric_limits<double>::epsilon() * 100.) {
      RCLCPP_WARN(getLogger(), "Quaternion is zero in RobotState representation. Setting to identity");
      values[3] = 0.;
      values[4] = 0.;
      values[5] = 0.;
      values[6] = 1.;
    }
    else {
      values[3] /= norm;
      values[4] /= norm;
      values[5] /= norm;
      values[6] /= norm;
    }
    return true;
  }
  else {
    return false;
  }
}

void FloatingJointModel::computeTransform(const double* joint_values, Eigen::Isometry3d& transform) const
{
  transform = Eigen::Isometry3d(
    Eigen::Translation3d(joint_values[0], joint_values[1], joint_values[2]) *
    Eigen::Quaterniond(joint_values[6], joint_values[3], joint_values[4], joint_values[5]).normalized());
}

void FloatingJointModel::computeVariablePositions(const Eigen::Isometry3d& transform, double* joint_values) const
{
  joint_values[0] = transform.translation().x();
  joint_values[1] = transform.translation().y();
  joint_values[2] = transform.translation().z();
  ASSERT_ISOMETRY(transform)  // unsanitized input, could contain non-isometry
  Eigen::Quaterniond q(transform.linear());
  joint_values[3] = q.x();
  joint_values[4] = q.y();
  joint_values[5] = q.z();
  joint_values[6] = q.w();
}

void FloatingJointModel::getVariableDefaultPositions(double* values) const
{
  for (uint32_t i = 0; i < 3; ++i) {
    values[i] = 0.;
  }

  values[3] = 0.;
  values[4] = 0.;
  values[5] = 0.;
  values[6] = 1.;
}
}  // namespace tobas
