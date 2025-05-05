#pragma once

#include "./frame_acc.hpp"
#include "./jntarray.hpp"
#include "./jntarray_vel.hpp"
#include "./utilities/utility.hpp"

namespace kdl
{
class JntArrayAcc
{
public:
  JntArray q;
  JntArray qdot;
  JntArray qdotdot;

  inline explicit JntArrayAcc();
  inline explicit JntArrayAcc(size_t nj);
  inline explicit JntArrayAcc(const JntArray& q, const JntArray& qdot, const JntArray& qdotdot);
  inline explicit JntArrayAcc(const JntArray& q, const JntArray& qdot);
  inline explicit JntArrayAcc(const JntArray& q);

  inline void resize(size_t nj);
  inline void setZero();

  // TODO: operators
};

inline JntArrayAcc::JntArrayAcc()
{
}

inline JntArrayAcc::JntArrayAcc(size_t nj) : q(nj), qdot(nj), qdotdot(nj)
{
}

inline JntArrayAcc::JntArrayAcc(const JntArray& _q, const JntArray& _qdot, const JntArray& _qdotdot)
  : q(_q), qdot(_qdot), qdotdot(_qdotdot)
{
  assert(q.rows() == qdot.rows() && qdot.rows() == qdotdot.rows());
}

inline JntArrayAcc::JntArrayAcc(const JntArray& _q, const JntArray& _qdot) : q(_q), qdot(_qdot), qdotdot(q.rows())
{
  assert(q.rows() == qdot.rows());
}

inline JntArrayAcc::JntArrayAcc(const JntArray& _q) : q(_q), qdot(q.rows()), qdotdot(q.rows())
{
}

inline void JntArrayAcc::resize(size_t nj)
{
  q.resize(nj);
  qdot.resize(nj);
  qdotdot.resize(nj);
}

inline void JntArrayAcc::setZero()
{
  q.setZero();
  qdot.setZero();
  qdotdot.setZero();
}
}  // namespace kdl
