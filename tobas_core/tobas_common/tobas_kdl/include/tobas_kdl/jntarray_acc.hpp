// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./frame_acc.hpp"
#include "./jntarray.hpp"
#include "./jntarray_vel.hpp"
#include "./utilities/utility.hpp"

namespace tobas
{
namespace kdl
{
class JntArrayAcc
{
public:
  JntArray q;
  JntArray qd;
  JntArray qdd;

  inline explicit JntArrayAcc();
  inline explicit JntArrayAcc(size_t nj);
  inline explicit JntArrayAcc(const JntArray& q, const JntArray& qd, const JntArray& qdd);
  inline explicit JntArrayAcc(const JntArray& q, const JntArray& qd);
  inline explicit JntArrayAcc(const JntArray& q);

  inline void resize(size_t nj);
  inline void setZero();

  // TODO: operators
};

inline JntArrayAcc::JntArrayAcc()
{
}

inline JntArrayAcc::JntArrayAcc(size_t nj) : q(nj), qd(nj), qdd(nj)
{
}

inline JntArrayAcc::JntArrayAcc(const JntArray& _q, const JntArray& _qd, const JntArray& _qdd)
  : q(_q), qd(_qd), qdd(_qdd)
{
  assert(q.size() == qd.size() && qd.size() == qdd.size());
}

inline JntArrayAcc::JntArrayAcc(const JntArray& _q, const JntArray& _qd) : q(_q), qd(_qd), qdd(q.size())
{
  assert(q.size() == qd.size());
}

inline JntArrayAcc::JntArrayAcc(const JntArray& _q) : q(_q), qd(q.size()), qdd(q.size())
{
}

inline void JntArrayAcc::resize(size_t nj)
{
  q.resize(nj);
  qd.resize(nj);
  qdd.resize(nj);
}

inline void JntArrayAcc::setZero()
{
  q.setZero();
  qd.setZero();
  qdd.setZero();
}
}  // namespace kdl
}  // namespace tobas
