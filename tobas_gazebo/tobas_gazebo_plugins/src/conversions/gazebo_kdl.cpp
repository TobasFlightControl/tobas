#include "../../include/tobas_gazebo_plugins/conversions/gazebo_kdl.hpp"
#include "../../include/tobas_gazebo_plugins/conversions/gazebo_eigen.hpp"

using namespace gz::math;

namespace gazebo
{
void vectorGazeboToKDL(const gz::math::Vector3d& g, kdl::Vector& k)
{
  k.x(g.X());
  k.y(g.Y());
  k.z(g.Z());
}

void vectorKDLToGazebo(const kdl::Vector& k, gz::math::Vector3d& g)
{
  g.X(k.x());
  g.Y(k.y());
  g.Z(k.z());
}

void rotationGazeboToKDL(const gz::math::Quaterniond& g, kdl::Rotation& k)
{
  k = kdl::Rotation::Quaternion(g.X(), g.Y(), g.Z(), g.W());
}

void rotationKDLToGazebo(const kdl::Rotation& k, gz::math::Quaterniond& g)
{
  gz::math::Matrix3d data;
  matrix3EigenToGazebo(k.data, data);
  g.SetFromMatrix(data);
}

void quaternionGazeboToKDL(const gz::math::Quaterniond& g, kdl::Quaternion& k)
{
  k.x = g.X();
  k.y = g.Y();
  k.z = g.Z();
  k.w = g.W();
}

void quaternionKDLToGazebo(const kdl::Quaternion& k, gz::math::Quaterniond& g)
{
  g.SetX(k.x);
  g.SetY(k.y);
  g.SetZ(k.z);
  g.SetW(k.w);
}

void poseGazeboToKDL(const gz::math::Pose3d& g, kdl::Frame& k)
{
  vectorGazeboToKDL(g.Pos(), k.p);
  rotationGazeboToKDL(g.Rot(), k.M);
}

void poseKDLToGazebo(const kdl::Frame& k, gz::math::Pose3d& g)
{
  vectorKDLToGazebo(k.p, g.Pos());
  rotationKDLToGazebo(k.M, g.Rot());
}
}  // namespace gazebo
