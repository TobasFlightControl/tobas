// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_kdl/tree_joint_axis_solver.hpp>
#include <tobas_kdl/tree_joint_parser.hpp>
#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_uadf/model.hpp>

#include "./frame_type.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
struct FrameTypeDetectionResult
{
  FrameType frame_type;
  std::string warning_message;
};

class FrameTypeDetector
{
public:
  explicit FrameTypeDetector(const uadf::Model& uadf, const kdl::Tree& tree);

  FrameTypeDetector(const FrameTypeDetector&) = delete;
  FrameTypeDetector& operator=(const FrameTypeDetector&) = delete;
  FrameTypeDetector(FrameTypeDetector&&) = delete;
  FrameTypeDetector& operator=(FrameTypeDetector&&) = delete;

  bool updateInternalDataStructures();
  FrameTypeDetectionResult determineFrameType();

private:
  static constexpr double kJntAxisParallelTol = st::deg2rad(5);  // [rad]

  bool isJntAxisAlwaysParallel(const std::string& link_name, const kdl::Vector& tar_axis, bool same_direction_only);
  bool isJntAxisAlwaysPerpendicular(const std::string& link_name, const kdl::Vector& tar_axis);
  bool allThrustJointAxesAlwaysParallel(const kdl::Vector& tar_axis, bool same_direction_only);
  bool allThrustJointAxesAlwaysPerpendicular(const kdl::Vector& tar_axis);
  bool allTiltJointAxesAlwaysParallel(const kdl::Vector& tar_axis, bool same_direction_only);
  bool eachTiltRotorAxesPerpendicular();
  bool allTiltRotorAxesPerpendicular();

  const uadf::Model& uadf_;
  const kdl::Tree& tree_;

  kdl::JntArray q_zeros_;
  kdl::TreeJointParser jnt_parser_;
  kdl::TreeJointAxisSolver axis_solver_;
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
