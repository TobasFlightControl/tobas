// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./base.hpp"

namespace tobas
{
namespace traj
{
/* ジャーク，加速度，速度の制約を満たした上で最短時間で目的地に到達する軌跡 (memo: 3-44) */
class TimeOptimalTrajectory : public TrajectoryGenerator
{
public:
  explicit TimeOptimalTrajectory(double p0, double pf, double max_jerk, double max_acc, double max_vel);

  TrajectoryPoint get(double t) const noexcept override;
  double duration() const noexcept override;

private:
  const double p0_;           // 初期位置
  const double pd_;           // p0を原点としたときの目的地 (符号なし)
  const int sign_;            // pdの符号
  double jm_, am_, vm_;       // リミット
  double t1_, t2_, t3_, t4_;  // 時刻

  /* pdを正としたときのp0に対する相対位置． */
  double p(double t) const noexcept;
  /* pdを正としたときの速度． */
  double v(double t) const noexcept;
  /* pdを正としたときの加速度． */
  double a(double t) const noexcept;
};
}  // namespace traj
}  // namespace tobas
