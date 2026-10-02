// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_uadf/model.hpp"

namespace tobas
{
namespace uadf
{
Model::Model()
{
}

void Model::clear()
{
  urdf.reset();

  thrusts.clear();
  control_surfaces.clear();
  tilts.clear();
}

std::expected<void, std::string> Model::validate() const
{
  // The URDF exists.
  if (!urdf) {
    return std::unexpected("URDF is null.");
  }

  // `"thrust"` must be an end joint.
  for (const auto& [thrust_joint_name, thrust] : thrusts) {
    const auto thrust_joint = urdf->getJoint(thrust_joint_name);
    if (!thrust_joint) {
      return std::unexpected("Thrust joint '" + thrust_joint_name + "' does not exist.");
    }

    const auto& thrust_link_name = thrust_joint->child_link_name;
    const auto thrust_link = urdf->getLink(thrust_link_name);
    if (!thrust_link) {
      return std::unexpected("Thrust link '" + thrust_link_name + "' does not exist.");
    }

    if (!thrust_link->child_joints.empty()) {
      return std::unexpected("Thrust link '" + thrust_link_name + "' must be an end link.");
    }
  }

  // `"cs"` must be an end joint.
  for (const auto& [cs_joint_name, cs] : control_surfaces) {
    const auto cs_joint = urdf->getJoint(cs_joint_name);
    if (!cs_joint) {
      return std::unexpected("CS joint '" + cs_joint_name + "' does not exist.");
    }

    const auto& cs_link_name = cs_joint->child_link_name;
    const auto cs_link = urdf->getLink(cs_link_name);
    if (!cs_link) {
      return std::unexpected("CS link '" + cs_link_name + "' does not exist.");
    }

    if (!cs_link->child_joints.empty()) {
      return std::unexpected("CS link '" + cs_link_name + "' must be an end link.");
    }
  }

  // The only joint after `"tilt"` must be one `"thrust"` joint.
  for (const auto& [tilt_joint_name, tilt] : tilts) {
    const auto tilt_joint = urdf->getJoint(tilt_joint_name);
    if (!tilt_joint) {
      return std::unexpected("Tilt joint '" + tilt_joint_name + "' does not exist.");
    }

    const auto& tilt_link_name = tilt_joint->child_link_name;
    const auto tilt_link = urdf->getLink(tilt_link_name);
    if (!tilt_link) {
      return std::unexpected("Tilt link '" + tilt_link_name + "' does not exist.");
    }

    if (tilt_link->child_joints.size() != 1) {
      return std::unexpected("Tilt link '" + tilt_link_name + "' must have one child joint.");
    }
    const auto& child_joint = tilt_link->child_joints.front();

    if (!thrusts.contains(child_joint->name)) {
      return std::unexpected("The joint type following '" + tilt_joint_name + "' must be 'thrust'.");
    }
  }

  return {};
}
}  // namespace uadf
}  // namespace tobas
