#pragma once

#include <boost/array.hpp>
#include <gazebo/gazebo.hh>

namespace gazebo
{
template <typename T>
bool allGreaterEqual(const ignition::math::Vector3<T>& v, T x)
{
  return v.X() >= x && v.Y() >= x && v.Z() >= x;
}

/* 3x3行列の対角要素を埋める． */
template <typename T>
void fillMatrix3Diag(boost::array<T, 9>& m, T v)
{
  m[0] = v;
  m[4] = v;
  m[8] = v;
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
}  // namespace gazebo
