#pragma once

#include "./accel.hpp"
#include "./wrench.hpp"
#include "./segmentinertia.hpp"

namespace kdl
{
/**
 * @brief 1つのセグメント対するヤコビアン．
 */
class SegmentJacobian
{
public:
  Vector linear;   // [m] (Revolute) or [-] (Prismatic)
  Vector angular;  // [-] (Revolute) or 0 (Prismatic)

  inline explicit SegmentJacobian();
  inline explicit SegmentJacobian(const Vector& linear, const Vector& angular);

  inline static SegmentJacobian Zero();

  inline void setZero();

  inline SegmentJacobian refPoint(const Vector& p) const;

  /* 関節空間の加速度からタスク空間の加速度を求める． */
  inline Accel accel(const double& qdd) const;

  /* 関節に働く力[N or Nm]を計算する． */
  inline double dot(const Wrench& rhs) const;
  /* 関節空間における慣性[kg m^2 or kg]を計算する． */
  inline double dot(const SegmentInertia& rhs) const;

  Eigen::Vector6d ravel() const;
};

inline SegmentJacobian::SegmentJacobian()
{
}

inline SegmentJacobian::SegmentJacobian(const Vector& _linear, const Vector& _angular)
  : linear(_linear), angular(_angular)
{
}

inline SegmentJacobian SegmentJacobian::Zero()
{
  return SegmentJacobian(Vector::Zero(), Vector::Zero());
}

inline void SegmentJacobian::setZero()
{
  linear.setZero();
  angular.setZero();
}

inline SegmentJacobian SegmentJacobian::refPoint(const Vector& p) const
{
  return SegmentJacobian(linear + angular * p, angular);
}

inline Accel SegmentJacobian::accel(const double& qdd) const
{
  return Accel(linear * qdd, angular * qdd);
}

inline double SegmentJacobian::dot(const Wrench& rhs) const
{
  return linear.dot(rhs.force) + angular.dot(rhs.torque);
}

inline double SegmentJacobian::dot(const SegmentInertia& rhs) const
{
  return linear.dot(rhs.linear) + angular.dot(rhs.angular);
}
}  // namespace kdl
