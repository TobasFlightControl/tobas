// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/double_spin_box.hpp>
#include <tobas_qt_tools/widgets/spin_box.hpp>

#include "./base.hpp"

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

  /* Pitch length at the propeller tip [m] */
  double pitchLength() const;

  /* Pitch angle at the propeller tip [rad] */
  double pitchAngle() const;

  /* Minimum blade chord length [m] */
  double minChord() const;

  /* Maximum blade chord length [m] */
  double maxChord() const;

  /* Average blade chord length [m] */
  double meanChord() const;

private:
  qt::SpinBox* num_blades_;
  qt::SpinBox* diameter_;
  qt::DoubleSpinBox* pitch_;
  qt::SpinBox* min_chord_;
  qt::SpinBox* max_chord_;
};
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
