// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <urdf/model.h>

#include <tobas_kdl/frame.hpp>
#include <tobas_kdl/joint.hpp>
#include <tobas_kdl/rigid_body_inertia.hpp>

namespace tobas
{
namespace kdl
{
void vectorKdlToUrdf(const Vector& _k, ::urdf::Vector3& _u);
void vectorUrdfToKdl(const ::urdf::Vector3& _u, Vector& _k);
::urdf::Vector3 vectorKdlToUrdf(const Vector& _k);
Vector vectorUrdfToKdl(const ::urdf::Vector3& _u);

void rotationKdlToUrdf(const Rotation& _k, ::urdf::Rotation& _u);
void rotationUrdfToKdl(const ::urdf::Rotation& _u, Rotation& _k);
::urdf::Rotation rotationKdlToUrdf(const Rotation& _k);
Rotation rotationUrdfToKdl(const ::urdf::Rotation& _u);

void poseKdlToUrdf(const Frame& _k, ::urdf::Pose& _u);
void poseUrdfToKdl(const ::urdf::Pose& _u, Frame& _k);
::urdf::Pose poseKdlToUrdf(const Frame& _k);
Frame poseUrdfToKdl(const ::urdf::Pose& _u);

void inertiaKdlToUrdf(const RigidBodyInertia& _k, ::urdf::Inertial& _u);
void inertiaUrdfToKdl(const ::urdf::Inertial& _u, RigidBodyInertia& _k);
::urdf::Inertial inertiaKdlToUrdf(const RigidBodyInertia& _k);
RigidBodyInertia inertiaUrdfToKdl(const ::urdf::Inertial& _u);

void jointKdlToUrdf(const Joint& _k, ::urdf::Joint& _u);
void jointUrdfToKdl(const ::urdf::Joint& _u, Joint& _k);
::urdf::Joint jointKdlToUrdf(const Joint& _k);
Joint jointUrdfToKdl(const ::urdf::Joint& _u);
}  // namespace kdl
}  // namespace tobas
