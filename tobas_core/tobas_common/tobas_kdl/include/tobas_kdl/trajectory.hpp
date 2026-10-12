// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./frames.hpp"

namespace tobas
{
namespace kdl
{
/** Generate a 3D cycloid. */
class CycloidGenerator3d
{
public:
  explicit CycloidGenerator3d();

  void generate(const kdl::Vector& p0, const kdl::Vector& pf, double T, double h, double k = 5.0);

  /**
   * Get the trajectory at time `t`.
   *
   * @param t Time from the start point.
   * @param r Rotation from the frame to view to the planning frame.
   * @param p Position at time `t`.
   * @param v Velocity at time `t`.
   * @param a Acceleration at time `t`.
   */
  void get(double t, const kdl::Rotation& r, kdl::Vector& p, kdl::Vector& v, kdl::Vector& a) const;

  /**
   * Get the trajectory at time `t`.
   *
   * @param t Time from the start point.
   * @param p Position at time `t`.
   * @param v Velocity at time `t`.
   * @param a Acceleration at time `t`.
   */
  void get(double t, kdl::Vector& p, kdl::Vector& v, kdl::Vector& a) const;

  /**
   * Get the trajectory at time `t`.
   *
   * @param t Time from the start point.
   * @param p Position at time `t`.
   * @param v Velocity at time `t`.
   */
  void get(double t, kdl::Vector& p, kdl::Vector& v) const;

  /**
   * Get the trajectory at time `t`.
   *
   * @param t Time from the start point.
   * @param p Position at time `t`.
   */
  void get(double t, kdl::Vector& p) const;

private:
  kdl::Vector p0_;
  kdl::Vector pf_;
  double T_;
  double h_;
  double k_;
  double TT_;
  double kk_;
  kdl::Vector p_diff_;
  const kdl::Rotation r0_;  ///< Identity matrix

  void getPos(double t, const kdl::Rotation& r, kdl::Vector& p) const;
  void getVel(double t, const kdl::Rotation& r, kdl::Vector& v) const;
  void getAcc(double t, const kdl::Rotation& r, kdl::Vector& a) const;

  double computeTheta(double t) const;
};
}  // namespace kdl
}  // namespace tobas
