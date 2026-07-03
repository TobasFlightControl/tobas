// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "../euler.hpp"
#include "../frames.hpp"
#include "../rotational_inertia.hpp"

namespace tobas
{
namespace kdl
{
void vectorFrdToFlu(const Vector& src, Vector& dst);
void vectorFluToFrd(const Vector& src, Vector& dst);
void vectorFrdToFlu(Vector& arg);
void vectorFluToFrd(Vector& arg);

void twistFrdToFlu(const Twist& src, Twist& dst);
void twistFluToFrd(const Twist& src, Twist& dst);
void twistFrdToFlu(Twist& arg);
void twistFluToFrd(Twist& arg);

void accelFrdToFlu(const Accel& src, Accel& dst);
void accelFluToFrd(const Accel& src, Accel& dst);
void accelFrdToFlu(Accel& arg);
void accelFluToFrd(Accel& arg);

/* Convert an inertia tensor from the FRD (Front-Right-Down) coordinate system
 * to the FLU (Front-Left-Up) coordinate system. (memo: 2-23) */
void rotInertiaFrdToFlu(const RotationalInertia& src, RotationalInertia& dst);
void rotInertiaFluToFrd(const RotationalInertia& src, RotationalInertia& dst);
void rotInertiaFrdToFlu(RotationalInertia& arg);
void rotInertiaFluToFrd(RotationalInertia& arg);

/* Compute the Euler angle change when both frames connected by a rotation are converted
 * from the FRD coordinate system to the FLU coordinate system. */
void eulerFrdToFlu(const Euler& src, Euler& dst);
void eulerFluToFrd(const Euler& src, Euler& dst);
void eulerFrdToFlu(Euler& arg);
void eulerFluToFrd(Euler& arg);

void rotationFrdToFlu(const kdl::Rotation& src, kdl::Rotation& dst);
void rotationFluToFrd(const kdl::Rotation& src, kdl::Rotation& dst);
void rotationFrdToFlu(kdl::Rotation& arg);
void rotationFluToFrd(kdl::Rotation& arg);

void frameFrdToFlu(const kdl::Frame& src, kdl::Frame& dst);
void frameFluToFrd(const kdl::Frame& src, kdl::Frame& dst);
void frameFrdToFlu(kdl::Frame& arg);
void frameFluToFrd(kdl::Frame& arg);
}  // namespace kdl
}  // namespace tobas
