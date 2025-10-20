#pragma once

#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

namespace gazebo
{
template <typename T>
bool allGreaterEqual(const gz::math::Vector3<T>& v, T x)
{
  return v.X() >= x && v.Y() >= x && v.Z() >= x;
}

/* FLU (Front-Left-Up) 座標系 (Gazebo) から FRD (Front-Right-Down) 座標系 (航空力学) に変換． */
template <typename T>
void FLU2FRD(gz::math::Vector3<T>& v)
{
  v.Y() = -v.Y();
  v.Z() = -v.Z();
}

/* FRD (Front-Right-Down) 座標系 (航空力学) から FLU (Front-Left-Up) 座標系 (Gazebo) に変換． */
template <typename T>
void FRD2FLU(gz::math::Vector3<T>& v)
{
  v.Y() = -v.Y();
  v.Z() = -v.Z();
}

/* 等価角軸ベクトルからクオータニオンを作成． */
gz::math::Quaterniond quaternionFromAngleAxis(const gz::math::Vector3d& w);

/* 3次元ベクトルの外積を表す歪対称行列を計算する． */
gz::math::Matrix3d skewMatrix(const gz::math::Vector3d& v);
}  // namespace gazebo
