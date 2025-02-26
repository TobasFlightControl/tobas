#pragma once

#include <urdf/model.h>

#include "../frame.hpp"
#include "../rigid_body_inertia.hpp"
#include "../joint.hpp"

namespace kdl
{
Vector toKdl(const urdf::Vector3& v);
Rotation toKdl(const urdf::Rotation& r);
Frame toKdl(const urdf::Pose& p);
RigidBodyInertia toKdl(const urdf::Inertial& i);
Joint toKdl(const urdf::Joint& jnt);
}  // namespace kdl
