// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/conversion/coordinates.hpp"

#include "tobas_kdl/rotational_inertia.hpp"

namespace tobas
{
namespace kdl
{
void vectorFrdToFlu(const Vector& _src, Vector& _dst)
{
  _dst.x(_src.x());
  _dst.y(-_src.y());
  _dst.z(-_src.z());
}

void vectorFluToFrd(const Vector& _src, Vector& _dst)
{
  vectorFrdToFlu(_src, _dst);
}

void vectorFrdToFlu(Vector& _arg)
{
  vectorFrdToFlu(_arg, _arg);
}

void vectorFluToFrd(Vector& _arg)
{
  vectorFluToFrd(_arg, _arg);
}

void twistFrdToFlu(const Twist& _src, Twist& _dst)
{
  vectorFrdToFlu(_src.vel, _dst.vel);
  vectorFrdToFlu(_src.rot, _dst.rot);
}

void twistFluToFrd(const Twist& _src, Twist& _dst)
{
  twistFrdToFlu(_src, _dst);
}

void twistFrdToFlu(Twist& _arg)
{
  twistFrdToFlu(_arg, _arg);
}

void twistFluToFrd(Twist& _arg)
{
  twistFluToFrd(_arg, _arg);
}

void accelFrdToFlu(const Accel& _src, Accel& _dst)
{
  vectorFrdToFlu(_src.linear, _dst.linear);
  vectorFrdToFlu(_src.angular, _dst.angular);
}

void accelFluToFrd(const Accel& _src, Accel& _dst)
{
  accelFrdToFlu(_src, _dst);
}

void accelFrdToFlu(Accel& _arg)
{
  accelFrdToFlu(_arg, _arg);
}

void accelFluToFrd(Accel& _arg)
{
  accelFluToFrd(_arg, _arg);
}

void rotInertiaFrdToFlu(const RotationalInertia& _src, RotationalInertia& _dst)
{
  _dst.data(0, 0) = _src.data(0, 0);   // xx
  _dst.data(0, 1) = -_src.data(0, 1);  // xy
  _dst.data(0, 2) = -_src.data(0, 2);  // xz
  _dst.data(1, 0) = -_src.data(1, 0);  // yx
  _dst.data(1, 1) = _src.data(1, 1);   // yy
  _dst.data(1, 2) = _src.data(1, 2);   // yz
  _dst.data(2, 0) = -_src.data(2, 0);  // zx
  _dst.data(2, 1) = _src.data(2, 1);   // zy
  _dst.data(2, 2) = _src.data(2, 2);   // zz
}

void rotInertiaFluToFrd(const RotationalInertia& _src, RotationalInertia& _dst)
{
  rotInertiaFrdToFlu(_src, _dst);
}

void rotInertiaFrdToFlu(RotationalInertia& _arg)
{
  rotInertiaFrdToFlu(_arg, _arg);
}

void rotInertiaFluToFrd(RotationalInertia& _arg)
{
  rotInertiaFluToFrd(_arg, _arg);
}

void eulerFrdToFlu(const Euler& _src, Euler& _dst)
{
  _dst.roll = _src.roll;
  _dst.pitch = -_src.pitch;
  _dst.yaw = -_src.yaw;
}

void eulerFluToFrd(const Euler& _src, Euler& _dst)
{
  eulerFrdToFlu(_src, _dst);
}

void eulerFrdToFlu(Euler& _arg)
{
  eulerFrdToFlu(_arg, _arg);
}

void eulerFluToFrd(Euler& _arg)
{
  eulerFluToFrd(_arg, _arg);
}

void rotationFrdToFlu(const kdl::Rotation& _src, kdl::Rotation& _dst)
{
  _dst.data(0, 0) = _src.data(0, 0);   // xx
  _dst.data(0, 1) = -_src.data(0, 1);  // xy
  _dst.data(0, 2) = -_src.data(0, 2);  // xz
  _dst.data(1, 0) = -_src.data(1, 0);  // yx
  _dst.data(1, 1) = _src.data(1, 1);   // yy
  _dst.data(1, 2) = _src.data(1, 2);   // yz
  _dst.data(2, 0) = -_src.data(2, 0);  // zx
  _dst.data(2, 1) = _src.data(2, 1);   // zy
  _dst.data(2, 2) = _src.data(2, 2);   // zz
}

void rotationFluToFrd(const kdl::Rotation& _src, kdl::Rotation& _dst)
{
  rotationFrdToFlu(_src, _dst);
}

void rotationFrdToFlu(kdl::Rotation& _arg)
{
  rotationFrdToFlu(_arg, _arg);
}

void rotationFluToFrd(kdl::Rotation& _arg)
{
  rotationFluToFrd(_arg, _arg);
}

void frameFrdToFlu(const kdl::Frame& _src, kdl::Frame& _dst)
{
  vectorFrdToFlu(_src.p, _dst.p);
  rotationFrdToFlu(_src.M, _dst.M);
}

void frameFluToFrd(const kdl::Frame& _src, kdl::Frame& _dst)
{
  frameFrdToFlu(_src, _dst);
}

void frameFrdToFlu(kdl::Frame& _arg)
{
  frameFrdToFlu(_arg, _arg);
}

void frameFluToFrd(kdl::Frame& _arg)
{
  frameFluToFrd(_arg, _arg);
}
}  // namespace kdl
}  // namespace tobas
