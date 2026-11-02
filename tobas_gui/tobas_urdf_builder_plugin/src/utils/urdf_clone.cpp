// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_urdf_builder_plugin/utils/urdf_clone.hpp"

namespace tobas::gui::ub::utils
{
::urdf::GeometrySharedPtr clone(const ::urdf::GeometrySharedPtr& _geometry)
{
  switch (_geometry->type) {
    case ::urdf::Geometry::BOX: {
      const auto box = ::urdf::dynamic_pointer_cast<::urdf::Box>(_geometry);
      return std::make_shared<::urdf::Box>(*box);
    }
    case ::urdf::Geometry::CYLINDER: {
      const auto cylinder = ::urdf::dynamic_pointer_cast<::urdf::Cylinder>(_geometry);
      return std::make_shared<::urdf::Cylinder>(*cylinder);
    }
    case ::urdf::Geometry::SPHERE: {
      const auto sphere = ::urdf::dynamic_pointer_cast<::urdf::Sphere>(_geometry);
      return std::make_shared<::urdf::Sphere>(*sphere);
    }
    case ::urdf::Geometry::CAPSULE: {
      const auto capsule = ::urdf::dynamic_pointer_cast<::urdf::Capsule>(_geometry);
      return std::make_shared<::urdf::Capsule>(*capsule);
    }
    case ::urdf::Geometry::MESH: {
      const auto mesh = ::urdf::dynamic_pointer_cast<::urdf::Mesh>(_geometry);
      return std::make_shared<::urdf::Mesh>(*mesh);
    }
    default: {
      throw;
    }
  }
}

::urdf::VisualSharedPtr clone(const ::urdf::VisualSharedPtr& _visual)
{
  if (!_visual) {
    return nullptr;
  }

  const auto res = std::make_shared<::urdf::Visual>(*_visual);
  res->geometry = clone(_visual->geometry);
  res->material = clone(_visual->material);
  return res;
}

::urdf::CollisionSharedPtr clone(const ::urdf::CollisionSharedPtr& _collision)
{
  if (!_collision) {
    return nullptr;
  }

  const auto res = std::make_shared<::urdf::Collision>(*_collision);
  res->geometry = clone(_collision->geometry);
  return res;
}

::urdf::JointCalibrationSharedPtr clone(const ::urdf::JointCalibrationSharedPtr& _calibration)
{
  if (!_calibration) {
    return nullptr;
  }

  const auto res = std::make_shared<::urdf::JointCalibration>(*_calibration);
  res->falling = clone(_calibration->falling);
  res->rising = clone(_calibration->rising);
  return res;
}

::urdf::JointSharedPtr clone(const ::urdf::JointSharedPtr& _joint)
{
  if (!_joint) {
    return nullptr;
  }

  const auto res = std::make_shared<::urdf::Joint>(*_joint);
  res->dynamics = clone(_joint->dynamics);
  res->limits = clone(_joint->limits);
  res->safety = clone(_joint->safety);
  res->calibration = clone(_joint->calibration);
  res->mimic = clone(_joint->mimic);
  return res;
}

::urdf::LinkSharedPtr clone(const ::urdf::LinkSharedPtr& _link)
{
  if (!_link) {
    return nullptr;
  }

  const auto res = std::make_shared<::urdf::Link>(*_link);
  res->inertial = clone(_link->inertial);
  res->visual = clone(_link->visual);
  res->collision = clone(_link->collision);
  res->parent_joint = clone(_link->parent_joint);

  res->collision_array.clear();
  for (const auto& collision : _link->collision_array) {
    res->collision_array.push_back(clone(collision));
  }

  res->visual_array.clear();
  for (const auto& visual : _link->visual_array) {
    res->visual_array.push_back(clone(visual));
  }

  res->child_links.clear();
  res->child_joints.clear();
  for (const auto& child_link : _link->child_links) {
    const auto child_link_clone = clone(child_link);
    res->child_links.push_back(child_link_clone);
    res->child_joints.push_back(child_link_clone->parent_joint);
  }

  return res;
}
}  // namespace tobas::gui::ub::utils
