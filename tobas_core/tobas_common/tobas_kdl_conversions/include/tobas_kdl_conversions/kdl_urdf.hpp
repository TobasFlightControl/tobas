#pragma once

#include <urdf/model.h>

#include <tobas_kdl/frame.hpp>
#include <tobas_kdl/joint.hpp>
#include <tobas_kdl/rigid_body_inertia.hpp>

namespace kdl
{
void vectorKdlToUrdf(const Vector& k, urdf::Vector3& u);
void vectorUrdfToKdl(const urdf::Vector3& u, Vector& k);
urdf::Vector3 vectorKdlToUrdf(const Vector& k);
Vector vectorUrdfToKdl(const urdf::Vector3& u);

void rotationKdlToUrdf(const Rotation& k, urdf::Rotation& u);
void rotationUrdfToKdl(const urdf::Rotation& u, Rotation& k);
urdf::Rotation rotationKdlToUrdf(const Rotation& k);
Rotation rotationUrdfToKdl(const urdf::Rotation& u);

void poseKdlToUrdf(const Frame& k, urdf::Pose& u);
void poseUrdfToKdl(const urdf::Pose& u, Frame& k);
urdf::Pose poseKdlToUrdf(const Frame& k);
Frame poseUrdfToKdl(const urdf::Pose& u);

void inertiaKdlToUrdf(const RigidBodyInertia& k, urdf::Inertial& u);
void inertiaUrdfToKdl(const urdf::Inertial& u, RigidBodyInertia& k);
urdf::Inertial inertiaKdlToUrdf(const RigidBodyInertia& k);
RigidBodyInertia inertiaUrdfToKdl(const urdf::Inertial& u);

void jointKdlToUrdf(const Joint& k, urdf::Joint& u);
void jointUrdfToKdl(const urdf::Joint& u, Joint& k);
urdf::Joint jointKdlToUrdf(const Joint& k);
Joint jointUrdfToKdl(const urdf::Joint& u);
}  // namespace kdl
