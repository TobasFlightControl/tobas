// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/conversion/coordinates.hpp"

#include "tobas_kdl/rotational_inertia.hpp"

namespace tobas
{
namespace kdl
{
void vectorFrdToFlu(const Vector& src, Vector& dst)
{
  dst.x(src.x());
  dst.y(-src.y());
  dst.z(-src.z());
}

void vectorFluToFrd(const Vector& src, Vector& dst)
{
  vectorFrdToFlu(src, dst);
}

void vectorFrdToFlu(Vector& arg)
{
  vectorFrdToFlu(arg, arg);
}

void vectorFluToFrd(Vector& arg)
{
  vectorFluToFrd(arg, arg);
}

void twistFrdToFlu(const Twist& src, Twist& dst)
{
  vectorFrdToFlu(src.vel, dst.vel);
  vectorFrdToFlu(src.rot, dst.rot);
}

void twistFluToFrd(const Twist& src, Twist& dst)
{
  twistFrdToFlu(src, dst);
}

void twistFrdToFlu(Twist& arg)
{
  twistFrdToFlu(arg, arg);
}

void twistFluToFrd(Twist& arg)
{
  twistFluToFrd(arg, arg);
}

void accelFrdToFlu(const Accel& src, Accel& dst)
{
  vectorFrdToFlu(src.linear, dst.linear);
  vectorFrdToFlu(src.angular, dst.angular);
}

void accelFluToFrd(const Accel& src, Accel& dst)
{
  accelFrdToFlu(src, dst);
}

void accelFrdToFlu(Accel& arg)
{
  accelFrdToFlu(arg, arg);
}

void accelFluToFrd(Accel& arg)
{
  accelFluToFrd(arg, arg);
}

void rotInertiaFrdToFlu(const RotationalInertia& src, RotationalInertia& dst)
{
  dst.data(0, 0) = src.data(0, 0);   // xx
  dst.data(0, 1) = -src.data(0, 1);  // xy
  dst.data(0, 2) = -src.data(0, 2);  // xz
  dst.data(1, 0) = -src.data(1, 0);  // yx
  dst.data(1, 1) = src.data(1, 1);   // yy
  dst.data(1, 2) = src.data(1, 2);   // yz
  dst.data(2, 0) = -src.data(2, 0);  // zx
  dst.data(2, 1) = src.data(2, 1);   // zy
  dst.data(2, 2) = src.data(2, 2);   // zz
}

void rotInertiaFluToFrd(const RotationalInertia& src, RotationalInertia& dst)
{
  rotInertiaFrdToFlu(src, dst);
}

void rotInertiaFrdToFlu(RotationalInertia& arg)
{
  rotInertiaFrdToFlu(arg, arg);
}

void rotInertiaFluToFrd(RotationalInertia& arg)
{
  rotInertiaFluToFrd(arg, arg);
}

void eulerFrdToFlu(const Euler& src, Euler& dst)
{
  dst.roll = src.roll;
  dst.pitch = -src.pitch;
  dst.yaw = -src.yaw;
}

void eulerFluToFrd(const Euler& src, Euler& dst)
{
  eulerFrdToFlu(src, dst);
}

void eulerFrdToFlu(Euler& arg)
{
  eulerFrdToFlu(arg, arg);
}

void eulerFluToFrd(Euler& arg)
{
  eulerFluToFrd(arg, arg);
}

void rotationFrdToFlu(const kdl::Rotation& src, kdl::Rotation& dst)
{
  dst.data(0, 0) = src.data(0, 0);   // xx
  dst.data(0, 1) = -src.data(0, 1);  // xy
  dst.data(0, 2) = -src.data(0, 2);  // xz
  dst.data(1, 0) = -src.data(1, 0);  // yx
  dst.data(1, 1) = src.data(1, 1);   // yy
  dst.data(1, 2) = src.data(1, 2);   // yz
  dst.data(2, 0) = -src.data(2, 0);  // zx
  dst.data(2, 1) = src.data(2, 1);   // zy
  dst.data(2, 2) = src.data(2, 2);   // zz
}

void rotationFluToFrd(const kdl::Rotation& src, kdl::Rotation& dst)
{
  rotationFrdToFlu(src, dst);
}

void rotationFrdToFlu(kdl::Rotation& arg)
{
  rotationFrdToFlu(arg, arg);
}

void rotationFluToFrd(kdl::Rotation& arg)
{
  rotationFluToFrd(arg, arg);
}

void frameFrdToFlu(const kdl::Frame& src, kdl::Frame& dst)
{
  vectorFrdToFlu(src.p, dst.p);
  rotationFrdToFlu(src.M, dst.M);
}

void frameFluToFrd(const kdl::Frame& src, kdl::Frame& dst)
{
  frameFrdToFlu(src, dst);
}

void frameFrdToFlu(kdl::Frame& arg)
{
  frameFrdToFlu(arg, arg);
}

void frameFluToFrd(kdl::Frame& arg)
{
  frameFluToFrd(arg, arg);
}
}  // namespace kdl
}  // namespace tobas
