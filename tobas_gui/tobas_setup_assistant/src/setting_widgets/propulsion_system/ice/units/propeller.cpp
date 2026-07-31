// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/propulsion_units/propeller.hpp"

#include <QVBoxLayout>

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_yaml_tools/convert/range.hpp>
#include <tobas_yaml_tools/format.hpp>

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
PropellerWidget::PropellerWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  num_blades_ = new ParamGetterWidget_SpinBox("Number of Blades");
  num_blades_->setMinimum(2);
  num_blades_->setMaximum(5);
  rows->addWidget(num_blades_);

  diameter_ = new ParamGetterWidget_SpinBox("Propeller Diameter");
  diameter_->setMinimum(1);
  diameter_->setSuffix(" inch");
  rows->addWidget(diameter_);

  pitch_length_neutoral_ = new ParamGetterWidget_DoubleSpinBox(
    "Propeller Pitch (Neutoral)", "The propeller pitch length when the pitch angle is 0°.");
  pitch_length_neutoral_->setDecimals(1);
  pitch_length_neutoral_->setMinimum(0.1);
  pitch_length_neutoral_->setSuffix(" inch");
  rows->addWidget(pitch_length_neutoral_);

  pitch_angle_limit_ = new ParamGetterWidget_DoubleRange(
    "Pitch Angle Limit", "Specify the range over which thrust varies linearly with propeller pitch angle.");
  pitch_angle_limit_->setDecimals(1);
  pitch_angle_limit_->setMinimum(-90);
  pitch_angle_limit_->setMaximum(+90);
  pitch_angle_limit_->setSuffix(" deg");
  rows->addWidget(pitch_angle_limit_);

  center_pitch_angle_ = new ParamGetterWidget_DoubleSpinBox("Center Pitch Angle");
  center_pitch_angle_->setDecimals(1);
  center_pitch_angle_->setMinimum(-90);
  center_pitch_angle_->setMaximum(+90);
  center_pitch_angle_->setSuffix(" deg");
  rows->addWidget(center_pitch_angle_);

  max_pitch_angle_rate_ = new ParamGetterWidget_SpinBox("Max Pitch Angle Rate");
  max_pitch_angle_rate_->setMinimum(0);
  max_pitch_angle_rate_->setSuffix(" dps");
  rows->addWidget(max_pitch_angle_rate_);

  min_chord_ = new ParamGetterWidget_SpinBox("Minimum Blade Chord Length", "Typically at the propeller tip.");
  min_chord_->setMinimum(1);
  min_chord_->setSuffix(" mm");
  rows->addWidget(min_chord_);

  max_chord_ = new ParamGetterWidget_SpinBox("Maximum Blade Chord Length", "");
  max_chord_->setMinimum(1);
  max_chord_->setSuffix(" mm");
  rows->addWidget(max_chord_);

  rows->addStretch();
}

const char* PropellerWidget::name() const
{
  return "Propeller";
}

void PropellerWidget::setToDefaults()
{
  num_blades_->setValue(2);
  diameter_->setValue(49);
  pitch_length_neutoral_->setValue(18);
  pitch_angle_limit_->setValue({ -10, 10 });
  center_pitch_angle_->setValue(0);
  max_pitch_angle_rate_->setValue(600);
  min_chord_->setValue(45);
  max_chord_->setValue(60);
}

bool PropellerWidget::isValid()
{
  const auto [min_pitch, max_pitch] = pitchAngleLimit();
  if (0.0 <= min_pitch || max_pitch <= 0.0) {
    qt::qWarnBox(this, "Pitch angle range must include 0.");
    return false;
  }

  const auto center_pitch = centerPitchAngle();
  if (center_pitch < min_pitch || max_pitch < center_pitch) {
    qt::qWarnBox(this, "Center pitch angle is out of its limit.");
    return false;
  }

  if (min_chord_->getValue() > max_chord_->getValue()) {
    qt::qWarnBox(this, "Blade chord range is invalid.");
    return false;
  }

  return true;
}

void PropellerWidget::copyFrom(const BaseSelectedLinkSettingWidget* src)
{
  const auto derived = qt::qConstPointerCast<PropellerWidget>(src);

  num_blades_->setValue(derived->num_blades_->getValue());
  diameter_->setValue(derived->diameter_->getValue());
  pitch_length_neutoral_->setValue(derived->pitch_length_neutoral_->getValue());
  pitch_angle_limit_->setValue(derived->pitch_angle_limit_->getValue());
  center_pitch_angle_->setValue(derived->center_pitch_angle_->getValue());
  max_pitch_angle_rate_->setValue(derived->max_pitch_angle_rate_->getValue());
  min_chord_->setValue(derived->min_chord_->getValue());
  max_chord_->setValue(derived->max_chord_->getValue());
}

YAML::Node PropellerWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[num_blades_->name()] = num_blades_->getValue();
  node[diameter_->name()] = diameter_->getValue();
  node[pitch_length_neutoral_->name()] = yaml::format(pitch_length_neutoral_->getValue());
  node[pitch_angle_limit_->name()] = pitch_angle_limit_->getValue();
  node[center_pitch_angle_->name()] = yaml::format(center_pitch_angle_->getValue());
  node[max_pitch_angle_rate_->name()] = max_pitch_angle_rate_->getValue();
  node[min_chord_->name()] = min_chord_->getValue();
  node[max_chord_->name()] = max_chord_->getValue();

  return node;
}

void PropellerWidget::load(const YAML::Node& node)
{
  num_blades_->setValue(node[num_blades_->name()].as<int>());
  diameter_->setValue(node[diameter_->name()].as<int>());
  pitch_length_neutoral_->setValue(node[pitch_length_neutoral_->name()].as<double>());
  pitch_angle_limit_->setValue(node[pitch_angle_limit_->name()].as<st::Range<double>>());
  center_pitch_angle_->setValue(node[center_pitch_angle_->name()].as<double>());
  max_pitch_angle_rate_->setValue(node[max_pitch_angle_rate_->name()].as<int>());
  min_chord_->setValue(node[min_chord_->name()].as<int>());
  max_chord_->setValue(node[max_chord_->name()].as<int>());
}

int PropellerWidget::numBlades() const
{
  return num_blades_->getValue();
}

double PropellerWidget::diameter() const
{
  return st::inch2meter(diameter_->getValue());
}

double PropellerWidget::radius() const
{
  return diameter() / 2;
}

double PropellerWidget::pitchLengthNeutoral() const
{
  return st::inch2meter(pitch_length_neutoral_->getValue());
}

double PropellerWidget::pitchAngleNeutoral() const
{
  return std::atan(pitchLengthNeutoral() / (M_PI * diameter()));
}

st::Range<double> PropellerWidget::pitchAngleLimit() const
{
  const auto lower = st::deg2rad(pitch_angle_limit_->min());
  const auto upper = st::deg2rad(pitch_angle_limit_->max());
  return { lower, upper };
}

double PropellerWidget::centerPitchAngle() const
{
  return st::deg2rad(center_pitch_angle_->getValue());
}

double PropellerWidget::maxPitchAngleRate() const
{
  return st::deg2rad(max_pitch_angle_rate_->getValue());
}

double PropellerWidget::minChord() const
{
  return min_chord_->getValue() * 1e-3;
}

double PropellerWidget::maxChord() const
{
  return max_chord_->getValue() * 1e-3;
}

double PropellerWidget::meanChord() const
{
  // Calculate the mean chord length simply as the average of the minimum and maximum values.
  return (minChord() + maxChord()) / 2;
}
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
