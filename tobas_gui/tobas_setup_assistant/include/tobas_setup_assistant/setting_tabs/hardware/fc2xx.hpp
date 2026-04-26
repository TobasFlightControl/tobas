// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./base.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace hw
{
class FC2xxWidget : public BaseHardwareWidget
{
  Q_OBJECT

public:
  explicit FC2xxWidget();

  const char* name() const override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  bool isValid() override;

  const char* hardwarePackage() const override;

  int imuUpdateRate() const override;
  double gyroNoiseDensity() const override;
  double gyroRandomWalk() const override;
  int gyroBiasCorrTime() const override;
  double accNoiseDensity() const override;
  double accRandomWalk() const override;
  int accBiasCorrTime() const override;

  int magUpdateRate() const override;
  double magNoiseStddev() const override;
  double magHardBiasNorm() const override;

  int presUpdateRate() const override;
  double presNoiseStddev() const override;

  int gnssUpdateRate() const override;
  double gnssHorizontalPositionAccuracy() const override;
  double gnssVerticalPositionAccuracy() const override;
  double gnssHorizontalVelocityStddev() const override;
  double gnssVerticalVelocityStddev() const override;

  int numPwmChannels() const override;
  int numDShotChannels() const override;
};
}  // namespace hw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
