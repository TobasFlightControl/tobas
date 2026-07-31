// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>
#include <QWidget>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace hw
{
class BaseHardwareWidget : public QWidget
{
  Q_OBJECT

public:
  virtual const char* name() const = 0;

  virtual YAML::Node dump() const = 0;
  virtual void load(const YAML::Node& node) = 0;

  virtual void setToDefaults() = 0;
  virtual bool isValid() = 0;

  virtual const char* hardwarePackage() const = 0;

  /* [rad/s/√Hz] */
  virtual double gyroNoiseDensity() const = 0;
  /* [rad/s^2/√Hz] */
  virtual double gyroRandomWalk() const = 0;
  /* [s] */
  virtual int gyroBiasCorrTime() const = 0;
  /* [m/s^2/√Hz] */
  virtual double accNoiseDensity() const = 0;
  /* [m/s^3/√Hz] */
  virtual double accRandomWalk() const = 0;
  /* [s] */
  virtual int accBiasCorrTime() const = 0;

  /* [Hz] */
  virtual int magUpdateRate() const = 0;
  /* [G] */
  virtual double magNoiseStddev() const = 0;
  /* [G] */
  virtual double magHardBiasNorm() const = 0;

  /* [Hz] */
  virtual int presUpdateRate() const = 0;
  /* [Pa] */
  virtual double presNoiseStddev() const = 0;

  /* [Hz] */
  virtual int gnssUpdateRate() const = 0;
  /* [m] */
  virtual double gnssHorizontalPositionAccuracy() const = 0;
  /* [m] */
  virtual double gnssVerticalPositionAccuracy() const = 0;
  /* [m/s] */
  virtual double gnssHorizontalVelocityStddev() const = 0;
  /* [m/s] */
  virtual double gnssVerticalVelocityStddev() const = 0;

  virtual int numPwmChannels() const = 0;
  virtual int numDShotChannels() const = 0;

  virtual bool hasRpmFilter() const = 0;
};
}  // namespace hw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
