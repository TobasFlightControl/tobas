#pragma once

#include "./jntarray.hpp"
#include "./framevel.hpp"
#include "./utilities/utility.hpp"

namespace KDL
{
class JntArrayVel
{
public:
  JntArray q;
  JntArray qdot;

  inline explicit JntArrayVel();
  inline explicit JntArrayVel(size_t nj);
  inline explicit JntArrayVel(const JntArray& q, const JntArray& qdot);
  inline explicit JntArrayVel(const JntArray& q);

  inline void resize(size_t nj);

  inline JntArrayVel operator+(const JntArrayVel& rhs) const;
  inline JntArrayVel operator-(const JntArrayVel& rhs) const;
  inline JntArrayVel operator*(const double& rhs) const;
  inline JntArrayVel operator*(const doubleVel& rhs) const;
  inline JntArrayVel operator/(const double& rhs) const;
  inline JntArrayVel operator/(const doubleVel& rhs) const;

  inline friend void setToZero(JntArrayVel& array);
};

inline JntArrayVel::JntArrayVel()
{
}

inline JntArrayVel::JntArrayVel(size_t nj) : q(nj), qdot(nj)
{
}

inline JntArrayVel::JntArrayVel(const JntArray& _q, const JntArray& _qdot) : q(_q), qdot(_qdot)
{
  assert(q.rows() == qdot.rows());
}

inline JntArrayVel::JntArrayVel(const JntArray& _q) : q(_q), qdot(q.rows())
{
}

inline void JntArrayVel::resize(size_t nj)
{
  q.resize(nj);
  qdot.resize(nj);
}

inline void setToZero(JntArrayVel& array)
{
  setToZero(array.q);
  setToZero(array.qdot);
}

inline JntArrayVel JntArrayVel::operator+(const JntArrayVel& rhs) const
{
  return JntArrayVel(q + rhs.q, qdot + rhs.qdot);
}

inline JntArrayVel JntArrayVel::operator-(const JntArrayVel& rhs) const
{
  return JntArrayVel(q - rhs.q, qdot - rhs.qdot);
}

inline JntArrayVel JntArrayVel::operator*(const double& rhs) const
{
  return JntArrayVel(q * rhs, qdot * rhs);
}

inline JntArrayVel JntArrayVel::operator*(const doubleVel& rhs) const
{
  return JntArrayVel(q * rhs.t, qdot * rhs.t + q * rhs.grad);
}

inline JntArrayVel JntArrayVel::operator/(const double& rhs) const
{
  assert(rhs != 0);
  return JntArrayVel(q / rhs, qdot / rhs);
}

inline JntArrayVel JntArrayVel::operator/(const doubleVel& rhs) const
{
  assert(rhs.t != 0);
  return JntArrayVel(q / rhs.t, qdot / rhs.t - q * (rhs.grad / sqr(rhs.t)));
}
}  // namespace KDL
