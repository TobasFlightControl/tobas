#pragma once

#include <urdf/model.h>

#include <tobas_kdl/frame.hpp>
#include <tobas_kdl/rigid_body_inertia.hpp>
#include <tobas_kdl/joint.hpp>

namespace kdl
{
Vector toKdl(const urdf::Vector3& v);
Rotation toKdl(const urdf::Rotation& r);
Frame toKdl(const urdf::Pose& p);
RigidBodyInertia toKdl(const urdf::Inertial& i);
Joint toKdl(const urdf::Joint& jnt);
}  // namespace kdl
