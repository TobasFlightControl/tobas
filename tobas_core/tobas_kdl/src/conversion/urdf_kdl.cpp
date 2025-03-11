#include "../include/tobas_kdl/conversion/urdf_kdl.hpp"

using namespace std;

namespace kdl
{
Vector toKdl(const urdf::Vector3& v)
{
  return Vector(v.x, v.y, v.z);
}

Rotation toKdl(const urdf::Rotation& r)
{
  return Rotation::Quaternion(r.x, r.y, r.z, r.w);
}

Frame toKdl(const urdf::Pose& p)
{
  return Frame(toKdl(p.rotation), toKdl(p.position));
}

RigidBodyInertia toKdl(const urdf::Inertial& i)
{
  const auto origin = toKdl(i.origin);

  // the mass is frame independent
  const auto& kdl_mass = i.mass;

  // kdl and urdf both specify the com position in the reference frame of the link
  const auto& kdl_com = origin.p;

  // kdl specifies the inertia matrix in the reference frame of the link,
  // while the urdf specifies the inertia matrix in the inertia reference frame
  const RotationalInertia urdf_inertia(i.ixx, i.iyy, i.izz, i.ixy, i.ixz, i.iyz);

  // Rotation operators are not defined for rotational inertia,
  // so we use the RigidBodyInertia operators (with com = 0) as a workaround
  const RigidBodyInertia kdl_inertia_wrt_com_workaround = origin.M * RigidBodyInertia(0, Vector::Zero(), urdf_inertia);

  // Note that the RigidBodyInertia constructor takes the 3d inertia wrt the com
  // while the getRotationalInertia method returns the 3d inertia wrt the frame origin
  // (but having com = Vector::Zero() in kdl_inertia_wrt_com_workaround they match)
  const RotationalInertia kdl_inertia_wrt_com = kdl_inertia_wrt_com_workaround.getRotationalInertia();

  return RigidBodyInertia(kdl_mass, kdl_com, kdl_inertia_wrt_com);
}

Joint toKdl(const urdf::Joint& jnt)
{
  const auto F_parent_jnt = toKdl(jnt.parent_to_joint_origin_transform);

  Joint res;
  res.name = jnt.name;

  if (jnt.type == urdf::Joint::FIXED)
  {
    res.type = Joint::FIXED;
  }
  else
  {
    if (jnt.type == urdf::Joint::REVOLUTE || jnt.type == urdf::Joint::CONTINUOUS)
      res.type = Joint::ROTATION;
    else if (jnt.type == urdf::Joint::PRISMATIC)
      res.type = Joint::TRANSLATION;
    else
      throw runtime_error("Unknown joint type of joint: " + jnt.name);

    res.origin = F_parent_jnt.p;
    res.axis(F_parent_jnt.M * toKdl(jnt.axis));

    if (jnt.dynamics)
    {
      res.damping = jnt.dynamics->damping;
      res.friction = jnt.dynamics->friction;
    }

    if (jnt.limits)
    {
      res.lower_limit = jnt.limits->lower;
      res.upper_limit = jnt.limits->upper;
      res.max_effort = jnt.limits->effort;
      res.max_velocity = jnt.limits->velocity;
    }
  }

  return res;
}
}  // namespace kdl
