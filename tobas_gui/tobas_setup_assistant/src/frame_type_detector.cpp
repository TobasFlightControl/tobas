// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/frame_type_detector.hpp"

#include <tobas_std_tools/check.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace
{
constexpr double kJntAxisParallelTol = st::deg2rad(5);  // [rad]
}  // namespace

FrameTypeDetector::FrameTypeDetector(const uadf::Model& uadf, const kdl::Tree& tree)
  : uadf_(uadf), tree_(tree), jnt_parser_(tree), axis_solver_(tree)
{
}

bool FrameTypeDetector::updateInternalDataStructures()
{
  q_zeros_ = kdl::JntArray::Zero(tree_.getNrOfJoints());

  if (!jnt_parser_.updateInternalDataStructures()) {
    return false;
  }
  if (!axis_solver_.updateInternalDataStructures()) {
    return false;
  }

  return true;
}

FrameTypeDetectionResult FrameTypeDetector::determineFrameType()
{
  constexpr char kIsNotSupported[] = "\nis not supported.";

  std::string msg = "Airframe\n\n";

  if (uadf_.control_surfaces.empty()) {
    msg += "  • which does not have fixed wings\n";

    if (uadf_.tilts.empty()) {
      msg += "  • which does not have any tilt rotors\n";

      if (uadf_.thrusts.size() < 3) {
        msg += "  • which has fewer than 3 propellers\n";
        return { FrameType::kUndefined, msg + kIsNotSupported };  // TODO: Two propellers might be controllable.
      }
      else {
        msg += "  • which has 3 or more propellers\n";

        if (allThrustJointAxesAlwaysParallel(kdl::Vector::UnitZ(), true)) {
          msg += "  • whose propeller rotation axes all point toward Z+\n";
          return { FrameType::kPlanarMulticopter, {} };  // TODO: Classify by manipulability.
        }
        else {
          msg += "  • which have propellers whose rotation axis can be oriented in a direction other than Z+\n";
          return { FrameType::kNonPlanarMulticopter, {} };  // TODO: Classify by manipulability.
        }
      }
    }
    else {
      msg += "  • which has at least one tilt rotors\n";

      if (eachTiltRotorAxesPerpendicular()) {
        msg += "  • which has each tilt axis perpendicular to its corresponding propeller rotation axis\n";

        if (allTiltRotorAxesPerpendicular()) {
          msg += "  • whose tilt axes are always perpendicular to all rotor axes in any combination\n";

          if (allTiltJointAxesAlwaysParallel(kdl::Vector::UnitY(), false)) {
            msg += "  • whose tilt axes are parallel to the Y axis\n";
            return { FrameType::kYAxisTiltMulticopter, {} };
          }
          else {
            msg += "  • whose tilt axes are not parallel to the Y axis\n";
            return { FrameType::kUndefined, msg + kIsNotSupported };
          }
        }
        else {
          msg += "  • whose tilt axis and propeller rotation axis may not be perpendicular to each other\n";
          return { FrameType::kRandomAxisTiltMulticopter, {} };
        }
      }
      else {
        msg += "  • which has a tilt axis that is not perpendicular to the propeller rotation axis\n";
        return { FrameType::kUndefined,
                 msg + kIsNotSupported };  // TODO: Support models whose tilt axes and rotation axes are not orthogonal.
      }
    }
  }
  else {
    msg += "  • which has fixed wings\n";
    return { FrameType::kUndefined, msg + kIsNotSupported };  // TODO: Support fixed wings.
  }
}

bool FrameTypeDetector::isJntAxisAlwaysParallel(
  const std::string& link_name,
  const kdl::Vector& tar_axis,
  bool same_direction_only)
{
  const auto seg_it = tree_.getSegment(link_name);
  const auto& elem = seg_it->second;

  // Return true if traversal reaches the root link without problems.
  if (seg_it == tree_.getRootSegment()) {
    return true;
  }

  // The necessary and sufficient condition is that all movable joint axes in the chain are parallel to the target
  // at some generalized coordinate configuration.
  const auto& joint = elem.segment.joint();
  if (joint.type != kdl::Joint::kFixed) {
    TOBAS_CHECK(axis_solver_.jntToCart(q_zeros_, link_name) == kdl::SolverI::kNoError);
    const auto& cur_axis = axis_solver_.getAxis();
    if (!cur_axis.isParallel(tar_axis, same_direction_only, kJntAxisParallelTol)) {
      return false;
    }
  }

  // Check the parent link.
  const auto& par_name = elem.parent->first;
  return isJntAxisAlwaysParallel(par_name, tar_axis, same_direction_only);
}

bool FrameTypeDetector::isJntAxisAlwaysPerpendicular(const std::string& link_name, const kdl::Vector& tar_axis)
{
  const auto seg_it = tree_.getSegment(link_name);
  const auto& elem = seg_it->second;

  const auto& joint = elem.segment.joint();
  if (joint.type != kdl::Joint::kRotation) {
    std::cerr << link_name << " does not have a rotation type joint." << std::endl;
    return false;
  }

  // Condition 1: The axis in question is perpendicular to the target at some generalized coordinate configuration.
  TOBAS_CHECK(axis_solver_.jntToCart(q_zeros_, link_name) == kdl::SolverI::kNoError);
  const auto& axis = axis_solver_.getAxis();
  if (!axis.isPerpendicular(tar_axis)) {
    return false;
  }

  // Condition 2: All other movable joint axes in the chain are parallel to the target.
  // Strictly speaking, a joint axis parallel to the target may be followed
  // by any number of joint axes perpendicular to the axis in question,
  // but this case is ignored because such a configuration is probably quite rare.
  const auto& par_name = elem.parent->first;
  return isJntAxisAlwaysParallel(par_name, tar_axis, false);
}

bool FrameTypeDetector::allThrustJointAxesAlwaysParallel(const kdl::Vector& tar_axis, bool same_direction_only)
{
  for (const auto& [joint_name, _] : uadf_.thrusts) {
    const auto& link_name = jnt_parser_.segmentName(joint_name);
    if (!isJntAxisAlwaysParallel(link_name, tar_axis, same_direction_only)) {
      return false;
    }
  }

  return true;
}

bool FrameTypeDetector::allThrustJointAxesAlwaysPerpendicular(const kdl::Vector& tar_axis)
{
  for (const auto& [joint_name, _] : uadf_.thrusts) {
    const auto& link_name = jnt_parser_.segmentName(joint_name);
    if (!isJntAxisAlwaysPerpendicular(link_name, tar_axis)) {
      return false;
    }
  }

  return true;
}

bool FrameTypeDetector::allTiltJointAxesAlwaysParallel(const kdl::Vector& tar_axis, bool same_direction_only)
{
  for (const auto& [joint_name, _] : uadf_.tilts) {
    const auto& link_name = jnt_parser_.segmentName(joint_name);
    if (!isJntAxisAlwaysParallel(link_name, tar_axis, same_direction_only)) {
      return false;
    }
  }

  return true;
}

bool FrameTypeDetector::eachTiltRotorAxesPerpendicular()
{
  for (const auto& [tilt_joint_name, _] : uadf_.tilts) {
    const auto tilt_joint_urdf = uadf_.urdf->getJoint(tilt_joint_name);
    const auto& tilt_link_name = tilt_joint_urdf->child_link_name;
    const auto tilt_link_urdf = uadf_.urdf->getLink(tilt_link_name);
    const auto& thrust_joint_urdf = tilt_link_urdf->child_joints.front();
    const auto& thrust_link_name = thrust_joint_urdf->child_link_name;

    const auto& thrust_elem = tree_.getSegment(thrust_link_name)->second;
    const auto& thrust_seg = thrust_elem.segment;
    const auto& thrust_joint_kdl = thrust_seg.joint();

    const auto& tilt_elem = tree_.getSegment(tilt_link_name)->second;
    const auto& tilt_seg = tilt_elem.segment;
    const auto& tilt_joint_kdl = tilt_seg.joint();

    const auto& p = tilt_joint_kdl.axis();  // Tilt axis as seen from the grandparent link.
    const auto& q =
      tilt_seg.frame().M * thrust_joint_kdl.axis();  // Rotor axis as seen from the parent link joint frame.

    if (!p.isPerpendicular(q)) {
      return false;
    }
  }

  return true;
}

bool FrameTypeDetector::allTiltRotorAxesPerpendicular()
{
  if (uadf_.tilts.empty()) {
    std::cerr << "No tilt joints exist." << std::endl;
    return false;
  }

  // Get one tilt axis.
  const auto& first_tilt_joint_name = uadf_.tilts.cbegin()->first;
  const auto& first_tilt_link_name = jnt_parser_.segmentName(first_tilt_joint_name);
  TOBAS_CHECK(axis_solver_.jntToCart(q_zeros_, first_tilt_link_name) == kdl::SolverI::kNoError);
  const auto first_tilt_joint_axis = axis_solver_.getAxis().clone();

  // It is sufficient for all tilt axes to be parallel to the first tilt axis
  // and for all rotor axes to be perpendicular to it.
  if (!allTiltJointAxesAlwaysParallel(first_tilt_joint_axis, false)) {
    return false;
  }
  if (!allThrustJointAxesAlwaysPerpendicular(first_tilt_joint_axis)) {
    return false;
  }

  return true;
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas
