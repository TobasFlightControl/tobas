#pragma once

#include "./frame.hpp"
#include "./twist.hpp"
#include "./accel.hpp"

namespace kdl
{
/**
 * @brief represents both linear and angular damping.
 */
class TaskSpaceDamping
{
public:
  Vector linear;   // [Ns/m]
  Vector angular;  // [Nms/rad]

  inline explicit TaskSpaceDamping();
  inline explicit TaskSpaceDamping(const Vector& linear, const Vector& angular);

  inline static TaskSpaceDamping Zero();

  inline void setZero();

  inline Twist operator*(const Frame& rhs);
  inline Accel operator*(const Twist& rhs);
};

inline TaskSpaceDamping::TaskSpaceDamping()
{
}

inline TaskSpaceDamping::TaskSpaceDamping(const Vector& _linear, const Vector& _angular)
  : linear(_linear), angular(_angular)
{
}

inline TaskSpaceDamping TaskSpaceDamping::Zero()
{
  return TaskSpaceDamping(Vector::Zero(), Vector::Zero());
}

inline void TaskSpaceDamping::setZero()
{
  linear.setZero();
  angular.setZero();
}

inline Twist TaskSpaceDamping::operator*(const Frame& rhs)
{
  return Twist(linear.hadamard(rhs.p), angular.hadamard(rhs.M.getRot()));
}

inline Accel TaskSpaceDamping::operator*(const Twist& rhs)
{
  return Accel(linear.hadamard(rhs.vel), angular.hadamard(rhs.rot));
}
}  // namespace kdl
