// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.
// Source: https://github.com/orocos/orocos_kinematics_dynamics/blob/HEAD/orocos_kdl/src/utilities/utility.h
// Modified by Tobas, Inc.

/*****************************************************************************
 *  \author
 *  	Erwin Aertbelien, Div. PMA, Dep. of Mech. Eng., K.U.Leuven
 *
 *  \version
 *		ORO_Geometry V0.2
 *
 *	\par History
 *		- $log$
 *
 *	\par Release
 *		$Id: utility.h,v 1.1.1.1.2.4 2003/07/18 14:58:36 psoetens Exp $
 *		$Name:  $
 *  \file
 *    Included by most lrl-files to provide some general
 *    functions and macro definitions.
 *
 *  \par history
 *   - changed layout of the comments to accommodate doxygen
 */

#pragma once

namespace tobas::kdl
{
/**
 * Auxiliary class for argument types (Trait-template class)
 *
 * Is used to pass doubles by value, and arbitrary objects by const reference.
 * This is TWICE as fast (2 x less memory access) and avoids bugs in VC6++ concerning
 * the assignment of the result of intrinsic functions to const double&-typed variables,
 * and optimization on.
 */
template <class T>
class TI
{
public:
  typedef const T& Arg;  //< Used for passing the element to a function.
};

template <>
class TI<double>
{
public:
  typedef double Arg;
};

template <>
class TI<int>
{
public:
  typedef int Arg;
};

inline constexpr double sqr(double _arg)
{
  return _arg * _arg;
}
}  // namespace tobas::kdl
