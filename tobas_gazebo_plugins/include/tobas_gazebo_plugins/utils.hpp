#pragma once

#include <gazebo/gazebo.hh>

namespace gazebo
{
template <typename T>
bool allGreaterEqual(const ignition::math::Vector3<T>& v, T x)
{
  return v.X() >= x && v.Y() >= x && v.Z() >= x;
}

/* NWU座標系(Gazebo)からNED座標系(航空力学)に変換． */
template <typename T>
void NWU2NED(ignition::math::Vector3<T>& v)
{
  v.Y() = -v.Y();
  v.Z() = -v.Z();
}

/* NED座標系(航空力学)からNWU座標系(Gazebo)に変換． */
template <typename T>
void NED2NWU(ignition::math::Vector3<T>& v)
{
  v.Y() = -v.Y();
  v.Z() = -v.Z();
}

/* 等価角軸ベクトルからクオータニオンを作成． */
ignition::math::Quaterniond angleAxisToQuaternion(const ignition::math::Vector3d& w);
}  // namespace gazebo
