// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl_conversions/kdl_urdf.hpp"

namespace tobas
{
namespace kdl
{
void vectorKdlToUrdf(const Vector& _k, ::urdf::Vector3& _u)
{
  _u.x = _k.x();
  _u.y = _k.y();
  _u.z = _k.z();
}

void vectorUrdfToKdl(const ::urdf::Vector3& _u, Vector& _k)
{
  _k.x(_u.x);
  _k.y(_u.y);
  _k.z(_u.z);
}

::urdf::Vector3 vectorKdlToUrdf(const Vector& _k)
{
  ::urdf::Vector3 u;
  vectorKdlToUrdf(_k, u);
  return u;
}

Vector vectorUrdfToKdl(const ::urdf::Vector3& _u)
{
  Vector k;
  vectorUrdfToKdl(_u, k);
  return k;
}

void rotationKdlToUrdf(const Rotation& _k, ::urdf::Rotation& _u)
{
  _k.getQuaternion(_u.x, _u.y, _u.z, _u.w);
}

void rotationUrdfToKdl(const ::urdf::Rotation& _u, Rotation& _k)
{
  _k = Rotation::Quaternion(_u.x, _u.y, _u.z, _u.w);
}

::urdf::Rotation rotationKdlToUrdf(const Rotation& _k)
{
  ::urdf::Rotation u;
  rotationKdlToUrdf(_k, u);
  return u;
}

Rotation rotationUrdfToKdl(const ::urdf::Rotation& _u)
{
  Rotation k;
  rotationUrdfToKdl(_u, k);
  return k;
}

void poseKdlToUrdf(const Frame& _k, ::urdf::Pose& _u)
{
  vectorKdlToUrdf(_k.p, _u.position);
  rotationKdlToUrdf(_k.M, _u.rotation);
}

void poseUrdfToKdl(const ::urdf::Pose& _u, Frame& _k)
{
  vectorUrdfToKdl(_u.position, _k.p);
  rotationUrdfToKdl(_u.rotation, _k.M);
}

::urdf::Pose poseKdlToUrdf(const Frame& _k)
{
  ::urdf::Pose u;
  poseKdlToUrdf(_k, u);
  return u;
}

Frame poseUrdfToKdl(const ::urdf::Pose& _u)
{
  Frame k;
  poseUrdfToKdl(_u, k);
  return k;
}

void inertiaKdlToUrdf(const RigidBodyInertia&, ::urdf::Inertial&)
{
  throw std::runtime_error("Not implemented yet.");  // TODO
}

void inertiaUrdfToKdl(const ::urdf::Inertial& _u, RigidBodyInertia& _k)
{
  const auto kdl_origin = poseUrdfToKdl(_u.origin);
  const RotationalInertia urdf_inertia(_u.ixx, _u.iyy, _u.izz, _u.ixy, _u.ixz, _u.iyz);
  _k = RigidBodyInertia(_u.mass, kdl_origin.p, kdl_origin.M * urdf_inertia);
}

::urdf::Inertial inertiaKdlToUrdf(const RigidBodyInertia& _k)
{
  ::urdf::Inertial u;
  inertiaKdlToUrdf(_k, u);
  return u;
}

RigidBodyInertia inertiaUrdfToKdl(const ::urdf::Inertial& _u)
{
  RigidBodyInertia k;
  inertiaUrdfToKdl(_u, k);
  return k;
}

void jointKdlToUrdf(const Joint&, ::urdf::Joint&)
{
  throw std::runtime_error("Not implemented yet.");  // TODO
}

void jointUrdfToKdl(const ::urdf::Joint& _u, Joint& _k)
{
  // Name
  _k.name = _u.name;

  if (_u.type == ::urdf::Joint::FIXED) {
    _k.type = Joint::kFixed;
  }
  else {
    // Type
    if (_u.type == ::urdf::Joint::REVOLUTE || _u.type == ::urdf::Joint::CONTINUOUS) {
      _k.type = Joint::kRotation;
    }
    else if (_u.type == ::urdf::Joint::PRISMATIC) {
      _k.type = Joint::kTranslation;
    }
    else {
      throw std::runtime_error("The type of joint '" + _u.name + "' is not supported by KDL.");
    }

    // Origin
    const auto F_parent_jnt = poseUrdfToKdl(_u.parent_to_joint_origin_transform);
    _k.origin = F_parent_jnt.p;

    // Axis
    const auto kdl_axis = vectorUrdfToKdl(_u.axis);
    _k.axis(F_parent_jnt.M * kdl_axis);

    // Dynamics
    if (_u.dynamics) {
      _k.damping = _u.dynamics->damping;
      _k.friction = _u.dynamics->friction;
    }

    // Limits
    if (_u.limits) {
      _k.lower_limit = _u.limits->lower;
      _k.upper_limit = _u.limits->upper;
      _k.max_effort = _u.limits->effort;
      _k.max_velocity = _u.limits->velocity;
    }
  }
}

::urdf::Joint jointKdlToUrdf(const Joint& _k)
{
  ::urdf::Joint u;
  jointKdlToUrdf(_k, u);
  return u;
}

Joint jointUrdfToKdl(const ::urdf::Joint& _u)
{
  Joint k;
  jointUrdfToKdl(_u, k);
  return k;
}
}  // namespace kdl
}  // namespace tobas
