// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./frames.hpp"

namespace tobas::kdl
{
/** Generate a 3D cycloid. */
class CycloidGenerator3d
{
public:
  explicit CycloidGenerator3d();

  void generate(const kdl::Vector& _p0, const kdl::Vector& _pf, double _t, double _h, double _k = 5.0);

  /**
   * Get the trajectory at time `t`.
   *
   * @param _t Time from the start point.
   * @param _r Rotation from the frame to view to the planning frame.
   * @param _p Position at time `t`.
   * @param _v Velocity at time `t`.
   * @param _a Acceleration at time `t`.
   */
  void get(double _t, const kdl::Rotation& _r, kdl::Vector& _p, kdl::Vector& _v, kdl::Vector& _a) const;

  /**
   * Get the trajectory at time `t`.
   *
   * @param _t Time from the start point.
   * @param _p Position at time `t`.
   * @param _v Velocity at time `t`.
   * @param _a Acceleration at time `t`.
   */
  void get(double _t, kdl::Vector& _p, kdl::Vector& _v, kdl::Vector& _a) const;

  /**
   * Get the trajectory at time `t`.
   *
   * @param _t Time from the start point.
   * @param _p Position at time `t`.
   * @param _v Velocity at time `t`.
   */
  void get(double _t, kdl::Vector& _p, kdl::Vector& _v) const;

  /**
   * Get the trajectory at time `t`.
   *
   * @param _t Time from the start point.
   * @param _p Position at time `t`.
   */
  void get(double _t, kdl::Vector& _p) const;

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

  void getPos(double _t, const kdl::Rotation& _r, kdl::Vector& _p) const;
  void getVel(double _t, const kdl::Rotation& _r, kdl::Vector& _v) const;
  void getAcc(double _t, const kdl::Rotation& _r, kdl::Vector& _a) const;

  double computeTheta(double _t) const;
};
}  // namespace tobas::kdl
