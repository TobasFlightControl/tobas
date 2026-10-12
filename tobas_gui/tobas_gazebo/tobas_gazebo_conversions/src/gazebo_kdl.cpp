// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gazebo_conversions/gazebo_kdl.hpp"

#include "tobas_gazebo_conversions/gazebo_eigen.hpp"

namespace tobas
{
namespace gazebo
{
void vectorGazeboToKDL(const gz::math::Vector3d& _g, kdl::Vector& _k)
{
  _k.x(_g.X());
  _k.y(_g.Y());
  _k.z(_g.Z());
}

void vectorKDLToGazebo(const kdl::Vector& _k, gz::math::Vector3d& _g)
{
  _g.Set(_k.x(), _k.y(), _k.z());
}

void rotationGazeboToKDL(const gz::math::Quaterniond& _g, kdl::Rotation& _k)
{
  _k = kdl::Rotation::Quaternion(_g.X(), _g.Y(), _g.Z(), _g.W());
}

void rotationKDLToGazebo(const kdl::Rotation& _k, gz::math::Quaterniond& _g)
{
  gz::math::Matrix3d data;
  matrix3EigenToGazebo(_k.data, data);
  _g.SetFromMatrix(data);
}

void quaternionGazeboToKDL(const gz::math::Quaterniond& _g, kdl::Quaternion& _k)
{
  _k.w = _g.W();
  _k.x = _g.X();
  _k.y = _g.Y();
  _k.z = _g.Z();
}

void quaternionKDLToGazebo(const kdl::Quaternion& _k, gz::math::Quaterniond& _g)
{
  _g.Set(_k.w, _k.x, _k.y, _k.z);
}

void poseGazeboToKDL(const gz::math::Pose3d& _g, kdl::Frame& _k)
{
  vectorGazeboToKDL(_g.Pos(), _k.p);
  rotationGazeboToKDL(_g.Rot(), _k.M);
}

void poseKDLToGazebo(const kdl::Frame& _k, gz::math::Pose3d& _g)
{
  vectorKDLToGazebo(_k.p, _g.Pos());
  rotationKDLToGazebo(_k.M, _g.Rot());
}
}  // namespace gazebo
}  // namespace tobas
