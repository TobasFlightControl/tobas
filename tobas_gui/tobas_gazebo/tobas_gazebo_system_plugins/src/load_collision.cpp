// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gazebo_system_plugins/load_collision.hpp"

#include <array>
#include <cmath>

#include <gz/common/Mesh.hh>
#include <gz/common/SubMesh.hh>
#include <gz/math/AxisAlignedBox.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Pose.hh>
#include <sdf/Box.hh>
#include <sdf/Capsule.hh>
#include <sdf/Cone.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Ellipsoid.hh>
#include <sdf/Geometry.hh>
#include <sdf/Mesh.hh>
#include <sdf/Sphere.hh>

namespace tobas
{
namespace gazebo
{
namespace
{
std::optional<gz::math::AxisAlignedBox> meshBounds(const sdf::Mesh& shape)
{
  // Bounds of the source mesh also enclose its convex hull.
  // Avoid running convex decomposition in the simulation thread just to compute bounds.
  auto source = shape;
  source.SetOptimization(sdf::MeshOptimization::NONE);
  const auto mesh = gz::sim::loadMesh(source);
  if (!mesh || mesh->VertexCount() == 0) {
    return std::nullopt;
  }

  auto min = mesh->Min();
  auto max = mesh->Max();
  if (!shape.Submesh().empty()) {
    const auto submesh = mesh->SubMeshByName(shape.Submesh()).lock();
    if (!submesh || submesh->VertexCount() == 0) {
      return std::nullopt;
    }
    min = submesh->Min();
    max = submesh->Max();
    if (shape.CenterSubmesh()) {
      const auto center = (min + max) / 2;
      min -= center;
      max -= center;
    }
  }

  // AxisAlignedBox orders the endpoints, including for negative mesh scales.
  return gz::math::AxisAlignedBox(min * shape.Scale(), max * shape.Scale());
}

std::optional<gz::math::AxisAlignedBox> geometryBounds(const sdf::Geometry& geometry)
{
  gz::math::Vector3d size;

  switch (geometry.Type()) {
    case sdf::GeometryType::BOX: {
      size = geometry.BoxShape()->Size();
      break;
    }
    case sdf::GeometryType::CYLINDER: {
      const auto shape = geometry.CylinderShape();
      size.Set(2 * shape->Radius(), 2 * shape->Radius(), shape->Length());
      break;
    }
    case sdf::GeometryType::SPHERE: {
      const auto diameter = 2 * geometry.SphereShape()->Radius();
      size.Set(diameter, diameter, diameter);
      break;
    }
    case sdf::GeometryType::CAPSULE: {
      const auto shape = geometry.CapsuleShape();
      const auto diameter = 2 * shape->Radius();
      size.Set(diameter, diameter, shape->Length() + diameter);
      break;
    }
    case sdf::GeometryType::ELLIPSOID: {
      size = 2 * geometry.EllipsoidShape()->Radii();
      break;
    }
    case sdf::GeometryType::CONE: {
      const auto shape = geometry.ConeShape();
      size.Set(2 * shape->Radius(), 2 * shape->Radius(), shape->Length());
      break;
    }
    case sdf::GeometryType::MESH: {
      return meshBounds(*geometry.MeshShape());
    }
    case sdf::GeometryType::EMPTY:
    case sdf::GeometryType::PLANE:
    case sdf::GeometryType::HEIGHTMAP:
    case sdf::GeometryType::POLYLINE: {
      return std::nullopt;
    }
    default: {
      throw;
    }
  }

  if (!size.IsFinite() || size.Min() <= 0.0) {
    return std::nullopt;
  }

  return gz::math::AxisAlignedBox(-size / 2, size / 2);
}

bool boxesOverlap(
  const gz::math::Pose3d& first_pose,
  const gz::math::Vector3d& first_size,
  const gz::math::Pose3d& second_pose,
  const gz::math::Vector3d& second_size)
{
  const std::array basis = { gz::math::Vector3d::UnitX, gz::math::Vector3d::UnitY, gz::math::Vector3d::UnitZ };
  std::array<gz::math::Vector3d, 3> first_axes, second_axes;
  for (size_t i = 0; i < 3; ++i) {
    first_axes[i] = first_pose.Rot().RotateVector(basis[i]);
    second_axes[i] = second_pose.Rot().RotateVector(basis[i]);
  }

  const auto offset = second_pose.Pos() - first_pose.Pos();

  const auto separated = [&](const gz::math::Vector3d& axis)
  {
    const auto length = axis.Length();
    if (length < 1e-12) {
      return false;
    }
    const auto direction = axis / length;
    double radius = 0.0;
    for (size_t i = 0; i < 3; ++i) {
      const auto tmp1 = first_size[i] * std::abs(first_axes[i].Dot(direction));
      const auto tmp2 = second_size[i] * std::abs(second_axes[i].Dot(direction));
      radius += (tmp1 + tmp2) / 2;
    }
    // Treat touching and sub-nanometer gaps as overlap to avoid roundoff holes.
    return std::abs(offset.Dot(direction)) > radius + 1e-9;
  };

  // Separating axis theorem: six face normals and nine edge cross products.
  for (size_t i = 0; i < 3; ++i) {
    if (separated(first_axes[i]) || separated(second_axes[i])) {
      return false;
    }
    for (size_t j = 0; j < 3; ++j) {
      if (separated(first_axes[i].Cross(second_axes[j]))) {
        return false;
      }
    }
  }

  return true;
}
}  // namespace

std::optional<std::string> checkLoadCollision(
  gz::sim::Entity model,
  const gz::math::Pose3d& load_pose,
  const gz::math::Vector3d& load_size,
  const gz::sim::EntityComponentManager& ecm)
{
  for (const auto entity : ecm.Descendants(model)) {
    if (!ecm.Component<gz::sim::components::Collision>(entity)) {
      continue;
    }

    const auto geometry = ecm.Component<gz::sim::components::Geometry>(entity);
    if (!geometry) {
      continue;
    }

    const auto bounds = geometryBounds(geometry->Data());
    if (!bounds) {
      continue;
    }

    const auto collision_pose = gz::sim::worldPose(entity, ecm);
    const gz::math::Pose3d box_pose(collision_pose.CoordPositionAdd(bounds->Center()), collision_pose.Rot());
    if (boxesOverlap(load_pose, load_size, box_pose, bounds->Size())) {
      const auto name = gz::sim::scopedName(entity, ecm, "::", false);
      return "Load overlaps or touches the bounding box of aircraft collision '" + name + "'.";
    }
  }

  return std::nullopt;  // No collision
}
}  // namespace gazebo
}  // namespace tobas
