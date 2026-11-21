// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/double_spin_box.hpp>
#include <tobas_qt_tools/widgets/spin_box.hpp>

#include "./base.hpp"
#include "tobas_setup_assistant/utils/double_range.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
class PropellerWidget : public BaseSelectedLinkSettingWidget
{
  Q_OBJECT

public:
  explicit PropellerWidget();

  const char* name() const override;
  bool isValid() override;
  void copyFrom(const BaseSelectedLinkSettingWidget* src) override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  /* Number of blades per propeller */
  int numBlades() const;

  /* Diameter of the propeller's rotational plane [m] */
  double diameter() const;

  /* Radius of the propeller's rotational plane [m] */
  double radius() const;

  /* Pitch length at the neutoral position [m] */
  double pitchLengthNeutoral() const;

  /* Pitch angle at the neutoral position [rad] */
  double pitchAngleNeutoral() const;

  /* Variable pitch angle limit around the neutoral position [rad] */
  st::Range<double> pitchAngleLimit() const;

  /* Center pitch angle for control [rad] */
  double centerPitchAngle() const;

  /* Maximum pitch angle rate [rad/s] */
  double maxPitchAngleRate() const;

  /* Minimum blade chord length [m] */
  double minChord() const;

  /* Maximum blade chord length [m] */
  double maxChord() const;

  /* Average blade chord length [m] */
  double meanChord() const;

private:
  qt::SpinBox* num_blades_;
  qt::SpinBox* diameter_;
  qt::DoubleSpinBox* pitch_length_neutoral_;
  DoubleRangeWidget* pitch_angle_limit_;
  qt::DoubleSpinBox* center_pitch_angle_;
  qt::SpinBox* max_pitch_angle_rate_;
  qt::SpinBox* min_chord_;
  qt::SpinBox* max_chord_;
};
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
