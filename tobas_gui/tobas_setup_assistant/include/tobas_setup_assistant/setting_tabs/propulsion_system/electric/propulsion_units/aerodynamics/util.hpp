// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <eigen3/Eigen/Core>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
static constexpr char kRpmColName[] = "RPM";
static constexpr char kThrustColName[] = "Thrust [N]";
static constexpr char kTorqueColName[] = "Torque [Nm]";
static constexpr char kUiucCtColName[] = "CT";
static constexpr char kUiucCpColName[] = "CP";

double motorConstFromThrustStand(const Eigen::VectorXd& _rpms, const Eigen::VectorXd& _thrusts);
double momentConstFromThrustStand(const Eigen::VectorXd& _thrusts, const Eigen::VectorXd& _torques);

double motorConstFromUiuc(const Eigen::VectorXd& _cts, double _d);
double momentConstFromUiuc(const Eigen::VectorXd& _cts, const Eigen::VectorXd& _cps, double _d);
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
