// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/hardware/fc1xx.hpp"

#include <tobas_std_tools/universal_constants.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace hw
{
FC1xxWidget::FC1xxWidget()
{
}

const char* FC1xxWidget::name() const
{
  return "Tobas FC1xx";
}

YAML::Node FC1xxWidget::dump() const
{
  return YAML::Node(YAML::NodeType::Map);
}

void FC1xxWidget::load(const YAML::Node&)
{
}

bool FC1xxWidget::isValid()
{
  return true;
}

const char* FC1xxWidget::hardwarePackage() const
{
  return "tobas_fc1xx_ros";
}

int FC1xxWidget::imuUpdateRate() const
{
  return 800;
}

double FC1xxWidget::gyroNoiseDensity() const
{
  return 0.011 * st::kDeg2Rad;  // ISM330DLC
}

double FC1xxWidget::gyroRandomWalk() const
{
  return 0.;  // TODO
}

int FC1xxWidget::gyroBiasCorrTime() const
{
  return 1000;  // TODO
}

double FC1xxWidget::accNoiseDensity() const
{
  return 1.7e-4 * st::kGravity;  // ISM330DLC
}

double FC1xxWidget::accRandomWalk() const
{
  return 0.;  // TODO
}

int FC1xxWidget::accBiasCorrTime() const
{
  return 300;  // TODO
}

int FC1xxWidget::magUpdateRate() const
{
  return 50;
}

double FC1xxWidget::magNoiseStddev() const
{
  return 4.6e-3;  // IIS2MDC
}

double FC1xxWidget::magHardBiasNorm() const
{
  return 0.03;  // IIS2MDCの最大値は6000nTだが，キャリブレーションを前提としてそれより低めに設定．
}

int FC1xxWidget::presUpdateRate() const
{
  return 50;
}

double FC1xxWidget::presNoiseStddev() const
{
  return 1.16;  // ILPS22QS (Table 23: FS = 1260, AVG = 32, ODR/4)
}

int FC1xxWidget::gnssUpdateRate() const
{
  return 20;
}

double FC1xxWidget::gnssHorizontalPositionAccuracy() const
{
  return 5.;  // 一般的な単独測位の精度
}

double FC1xxWidget::gnssVerticalPositionAccuracy() const
{
  return 10.;  // 一般的な単独測位の精度
}

double FC1xxWidget::gnssHorizontalVelocityStddev() const
{
  return 0.05;  // FIXME: 精度 (Accuracy) と標準偏差は異なる
}

double FC1xxWidget::gnssVerticalVelocityStddev() const
{
  return 0.05;  // FIXME: 精度 (Accuracy) と標準偏差は異なる
}

int FC1xxWidget::numPwmChannels() const
{
  return 8;
}

int FC1xxWidget::numDShotChannels() const
{
  return 8;
}
}  // namespace hw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
