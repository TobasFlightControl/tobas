#pragma once

#include "./frame.hpp"
#include "./accel.hpp"

namespace kdl
{
/**
 * @brief represents both linear and angular stiffness.
 */
class TaskSpaceStiffness
{
public:
  Vector linear;   // [N/m]
  Vector angular;  // [Nm/rad] 等価角軸ベクトルに対する弾性係数

  inline explicit TaskSpaceStiffness();
  inline explicit TaskSpaceStiffness(const Vector& linear, const Vector& angular);

  inline static TaskSpaceStiffness Zero();

  inline void setZero();

  inline Accel operator*(const Frame& rhs);
};

inline TaskSpaceStiffness::TaskSpaceStiffness()
{
}

inline TaskSpaceStiffness::TaskSpaceStiffness(const Vector& _linear, const Vector& _angular)
  : linear(_linear), angular(_angular)
{
}

inline TaskSpaceStiffness TaskSpaceStiffness::Zero()
{
  return TaskSpaceStiffness(Vector::Zero(), Vector::Zero());
}

inline void TaskSpaceStiffness::setZero()
{
  linear.setZero();
  angular.setZero();
}

inline Accel TaskSpaceStiffness::operator*(const Frame& rhs)
{
  return Accel(linear.hadamard(rhs.p), angular.hadamard(rhs.M.getRot()));
}
}  // namespace kdl
