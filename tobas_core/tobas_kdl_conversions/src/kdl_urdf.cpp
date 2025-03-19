#include "../include/tobas_kdl_conversions/kdl_urdf.hpp"

using namespace std;

namespace kdl
{
void vectorKdlToUrdf(const Vector& k, urdf::Vector3& u)
{
  u.x = k.x();
  u.y = k.y();
  u.z = k.z();
}

void vectorUrdfToKdl(const urdf::Vector3& u, Vector& k)
{
  k.x(u.x);
  k.y(u.y);
  k.z(u.z);
}

urdf::Vector3 vectorKdlToUrdf(const Vector& k)
{
  urdf::Vector3 u;
  vectorKdlToUrdf(k, u);
  return u;
}

Vector vectorUrdfToKdl(const urdf::Vector3& u)
{
  Vector k;
  vectorUrdfToKdl(u, k);
  return k;
}

void rotationKdlToUrdf(const Rotation& k, urdf::Rotation& u)
{
  k.getQuaternion(u.x, u.y, u.z, u.w);
}

void rotationUrdfToKdl(const urdf::Rotation& u, Rotation& k)
{
  k = Rotation::Quaternion(u.x, u.y, u.z, u.w);
}

urdf::Rotation rotationKdlToUrdf(const Rotation& k)
{
  urdf::Rotation u;
  rotationKdlToUrdf(k, u);
  return u;
}

Rotation rotationUrdfToKdl(const urdf::Rotation& u)
{
  Rotation k;
  rotationUrdfToKdl(u, k);
  return k;
}

void poseKdlToUrdf(const Frame& k, urdf::Pose& u)
{
  vectorKdlToUrdf(k.p, u.position);
  rotationKdlToUrdf(k.M, u.rotation);
}

void poseUrdfToKdl(const urdf::Pose& u, Frame& k)
{
  vectorUrdfToKdl(u.position, k.p);
  rotationUrdfToKdl(u.rotation, k.M);
}

urdf::Pose poseKdlToUrdf(const Frame& k)
{
  urdf::Pose u;
  poseKdlToUrdf(k, u);
  return u;
}

Frame poseUrdfToKdl(const urdf::Pose& u)
{
  Frame k;
  poseUrdfToKdl(u, k);
  return k;
}

void inertiaKdlToUrdf(const RigidBodyInertia&, urdf::Inertial&)
{
  throw runtime_error("Not implemented yet.");  // TODO
}

void inertiaUrdfToKdl(const urdf::Inertial& u, RigidBodyInertia& k)
{
  const auto kdl_origin = poseUrdfToKdl(u.origin);
  const RotationalInertia urdf_inertia(u.ixx, u.iyy, u.izz, u.ixy, u.ixz, u.iyz);
  k = RigidBodyInertia(u.mass, kdl_origin.p, kdl_origin.M * urdf_inertia);
}

urdf::Inertial inertiaKdlToUrdf(const RigidBodyInertia& k)
{
  urdf::Inertial u;
  inertiaKdlToUrdf(k, u);
  return u;
}

RigidBodyInertia inertiaUrdfToKdl(const urdf::Inertial& u)
{
  RigidBodyInertia k;
  inertiaUrdfToKdl(u, k);
  return k;
}

void jointKdlToUrdf(const Joint&, urdf::Joint&)
{
  throw runtime_error("Not implemented yet.");  // TODO
}

void jointUrdfToKdl(const urdf::Joint& u, Joint& k)
{
  // Name
  k.name = u.name;

  if (u.type == urdf::Joint::FIXED)
  {
    k.type = Joint::FIXED;
  }
  else
  {
    // Type
    if (u.type == urdf::Joint::REVOLUTE || u.type == urdf::Joint::CONTINUOUS)
      k.type = Joint::ROTATION;
    else if (u.type == urdf::Joint::PRISMATIC)
      k.type = Joint::TRANSLATION;
    else
      throw runtime_error("Unknown joint type of joint: " + u.name);

    // Origin
    const auto F_parent_jnt = poseUrdfToKdl(u.parent_to_joint_origin_transform);
    k.origin = F_parent_jnt.p;

    // Axis
    const auto kdl_axis = vectorUrdfToKdl(u.axis);
    k.axis(F_parent_jnt.M * kdl_axis);

    // Dynamics
    if (u.dynamics)
    {
      k.damping = u.dynamics->damping;
      k.friction = u.dynamics->friction;
    }

    // Limits
    if (u.limits)
    {
      k.lower_limit = u.limits->lower;
      k.upper_limit = u.limits->upper;
      k.max_effort = u.limits->effort;
      k.max_velocity = u.limits->velocity;
    }
  }
}

urdf::Joint jointKdlToUrdf(const Joint& k)
{
  urdf::Joint u;
  jointKdlToUrdf(k, u);
  return u;
}

Joint jointUrdfToKdl(const urdf::Joint& u)
{
  Joint k;
  jointUrdfToKdl(u, k);
  return k;
}
}  // namespace kdl
