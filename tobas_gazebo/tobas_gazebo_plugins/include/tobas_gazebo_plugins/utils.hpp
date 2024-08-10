#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
template <typename T>
bool allGreaterEqual(const gz::math::Vector3<T>& v, T x)
{
  return v.X() >= x && v.Y() >= x && v.Z() >= x;
}

/* NWU座標系(Gazebo)からNED座標系(航空力学)に変換． */
template <typename T>
void NWU2NED(gz::math::Vector3<T>& v)
{
  v.Y() = -v.Y();
  v.Z() = -v.Z();
}

/* NED座標系(航空力学)からNWU座標系(Gazebo)に変換． */
template <typename T>
void NED2NWU(gz::math::Vector3<T>& v)
{
  v.Y() = -v.Y();
  v.Z() = -v.Z();
}

/* 等価角軸ベクトルからクオータニオンを作成． */
gz::math::Quaterniond angleAxisToQuaternion(const gz::math::Vector3d& w);

/* 3次元ベクトルの外積を表す歪対称行列を計算する． */
gz::math::Matrix3d skewMatrix(const gz::math::Vector3d& v);

/* ロボット全体の質量を計算する． */
double computeTotalMass(const physics::ModelPtr& model);
}  // namespace gazebo
