// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/hardware/fc2xx.hpp"

#include <tobas_std_tools/universal_constants.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace hw
{
FC2xxWidget::FC2xxWidget()
{
}

const char* FC2xxWidget::name() const
{
  return "Tobas FC2xx";
}

YAML::Node FC2xxWidget::dump() const
{
  return YAML::Node(YAML::NodeType::Map);
}

void FC2xxWidget::load(const YAML::Node&)
{
}

bool FC2xxWidget::isValid()
{
  return true;
}

const char* FC2xxWidget::hardwarePackage() const
{
  return "tobas_fc2xx_ros";
}

double FC2xxWidget::gyroNoiseDensity() const
{
  return 0.011 * st::kDeg2Rad;  // ISM330DLC
}

double FC2xxWidget::gyroRandomWalk() const
{
  return 0.;  // TODO
}

int FC2xxWidget::gyroBiasCorrTime() const
{
  return 1000;  // TODO
}

double FC2xxWidget::accNoiseDensity() const
{
  return 1.7e-4 * st::kGravity;  // ISM330DLC
}

double FC2xxWidget::accRandomWalk() const
{
  return 0.;  // TODO
}

int FC2xxWidget::accBiasCorrTime() const
{
  return 300;  // TODO
}

int FC2xxWidget::magUpdateRate() const
{
  return 50;
}

double FC2xxWidget::magNoiseStddev() const
{
  return 4.6e-3;  // IIS2MDC
}

double FC2xxWidget::magHardBiasNorm() const
{
  return 0.03;  // The IIS2MDC maximum is 6000 nT, but set this lower assuming calibration.
}

int FC2xxWidget::presUpdateRate() const
{
  return 50;
}

double FC2xxWidget::presNoiseStddev() const
{
  return 1.16;  // ILPS22QS (Table 23: FS = 1260, AVG = 32, ODR/4)
}

int FC2xxWidget::gnssUpdateRate() const
{
  return 20;
}

double FC2xxWidget::gnssHorizontalPositionAccuracy() const
{
  return 5.;  // Typical standalone positioning accuracy.
}

double FC2xxWidget::gnssVerticalPositionAccuracy() const
{
  return 10.;  // Typical standalone positioning accuracy.
}

double FC2xxWidget::gnssHorizontalVelocityStddev() const
{
  return 0.05;  // FIXME: Accuracy and standard deviation are different.
}

double FC2xxWidget::gnssVerticalVelocityStddev() const
{
  return 0.05;  // FIXME: Accuracy and standard deviation are different.
}

int FC2xxWidget::numPwmChannels() const
{
  return 8;
}

int FC2xxWidget::numDShotChannels() const
{
  return 8;
}

bool FC2xxWidget::hasRpmFilter() const
{
  return true;
}
}  // namespace hw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
