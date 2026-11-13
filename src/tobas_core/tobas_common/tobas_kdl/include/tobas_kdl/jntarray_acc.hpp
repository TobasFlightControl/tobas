// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.
// Source: https://github.com/orocos/orocos_kinematics_dynamics/blob/HEAD/orocos_kdl/src/jntarrayacc.hpp
// Modified by Tobas, Inc.

// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// Version: 1.0
// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.

// You should have received a copy of the GNU General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#pragma once

#include <cassert>
#include <cstddef>

#include "./jntarray.hpp"

namespace tobas::kdl
{
class JntArrayAcc
{
public:
  JntArray q;
  JntArray qd;
  JntArray qdd;

  inline explicit JntArrayAcc();
  inline explicit JntArrayAcc(size_t _nj);
  inline explicit JntArrayAcc(const JntArray& _q, const JntArray& _qd, const JntArray& _qdd);
  inline explicit JntArrayAcc(const JntArray& _q, const JntArray& _qd);
  inline explicit JntArrayAcc(const JntArray& _q);

  inline void resize(size_t _nj);
  inline void setZero();

  // TODO: operators
};

inline JntArrayAcc::JntArrayAcc()
{
}

inline JntArrayAcc::JntArrayAcc(size_t _nj) : q(_nj), qd(_nj), qdd(_nj)
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

inline void JntArrayAcc::resize(size_t _nj)
{
  q.resize(_nj);
  qd.resize(_nj);
  qdd.resize(_nj);
}

inline void JntArrayAcc::setZero()
{
  q.setZero();
  qd.setZero();
  qdd.setZero();
}
}  // namespace tobas::kdl
