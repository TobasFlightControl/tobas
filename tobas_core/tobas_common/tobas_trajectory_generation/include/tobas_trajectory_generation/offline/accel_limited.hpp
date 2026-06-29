// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./base.hpp"

namespace tobas
{
namespace traj
{
/* 加速度，速度の制約を満たした上で最短時間で目的地に到達する軌跡 (memo: 3-50) */
class AccelLimitedTrajectory : public TrajectoryGenerator
{
public:
  explicit AccelLimitedTrajectory(double p0, double v0, double pf, double vf, double max_acc);

  TrajectoryPoint get(double t) const noexcept override;
  double duration() const noexcept override;

private:
  const double p0_, v0_;  // 初期状態
  const double pf_, vf_;  // 目標状態
  const double am_;       // リミット

  double s_;   // スイッチング曲線
  double ts_;  // スイッチング時刻
  double tf_;  // 到達時刻
};
}  // namespace traj
}  // namespace tobas
