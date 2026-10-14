// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "../euler.hpp"
#include "../frames.hpp"
#include "../rotational_inertia.hpp"

namespace tobas::kdl
{
void vectorFrdToFlu(const Vector& _src, Vector& _dst);
void vectorFluToFrd(const Vector& _src, Vector& _dst);
void vectorFrdToFlu(Vector& _arg);
void vectorFluToFrd(Vector& _arg);

void twistFrdToFlu(const Twist& _src, Twist& _dst);
void twistFluToFrd(const Twist& _src, Twist& _dst);
void twistFrdToFlu(Twist& _arg);
void twistFluToFrd(Twist& _arg);

void accelFrdToFlu(const Accel& _src, Accel& _dst);
void accelFluToFrd(const Accel& _src, Accel& _dst);
void accelFrdToFlu(Accel& _arg);
void accelFluToFrd(Accel& _arg);

/**
 * Convert an inertia tensor from the FRD (Front-Right-Down) coordinate system
 * to the FLU (Front-Left-Up) coordinate system. (memo: 2-23)
 */
void rotInertiaFrdToFlu(const RotationalInertia& _src, RotationalInertia& _dst);
void rotInertiaFluToFrd(const RotationalInertia& _src, RotationalInertia& _dst);
void rotInertiaFrdToFlu(RotationalInertia& _arg);
void rotInertiaFluToFrd(RotationalInertia& _arg);

/**
 * Compute the Euler angle change when both frames connected by a rotation are converted
 * from the FRD coordinate system to the FLU coordinate system.
 */
void eulerFrdToFlu(const Euler& _src, Euler& _dst);
void eulerFluToFrd(const Euler& _src, Euler& _dst);
void eulerFrdToFlu(Euler& _arg);
void eulerFluToFrd(Euler& _arg);

void rotationFrdToFlu(const kdl::Rotation& _src, kdl::Rotation& _dst);
void rotationFluToFrd(const kdl::Rotation& _src, kdl::Rotation& _dst);
void rotationFrdToFlu(kdl::Rotation& _arg);
void rotationFluToFrd(kdl::Rotation& _arg);

void frameFrdToFlu(const kdl::Frame& _src, kdl::Frame& _dst);
void frameFluToFrd(const kdl::Frame& _src, kdl::Frame& _dst);
void frameFrdToFlu(kdl::Frame& _arg);
void frameFluToFrd(kdl::Frame& _arg);
}  // namespace tobas::kdl
