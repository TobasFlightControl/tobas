// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/propulsion_units/propeller.hpp"

#include <QVBoxLayout>

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>
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
namespace
{
// Keep the existing serialized keys independent of the form labels.
constexpr char kNumBladesKey[] = "Number of Blades";
constexpr char kDiameterKey[] = "Propeller Diameter";
constexpr char kPitchLengthNeutoralKey[] = "Propeller Pitch (Neutoral)";
constexpr char kPitchAngleLimitKey[] = "Pitch Angle Limit";
constexpr char kCenterPitchAngleKey[] = "Center Pitch Angle";
constexpr char kMaxPitchAngleRateKey[] = "Max Pitch Angle Rate";
constexpr char kMinChordKey[] = "Minimum Blade Chord Length";
constexpr char kMaxChordKey[] = "Maximum Blade Chord Length";
}  // namespace

PropellerWidget::PropellerWidget()
{
  const auto form = new qt::FormLayout();
  setLayout(form);

  num_blades_ = new qt::SpinBox();
  num_blades_->setMinimum(2);
  num_blades_->setMaximum(5);
  num_blades_->setValue(2);
  form->addRow(kNumBladesKey, num_blades_);
  form->labelForField(num_blades_)->setToolTip(num_blades_->toolTip());

  diameter_ = new qt::SpinBox();
  diameter_->setMinimum(1);
  diameter_->setValue(49);
  diameter_->setSuffix(" inch");
  form->addRow(kDiameterKey, diameter_);
  form->labelForField(diameter_)->setToolTip(diameter_->toolTip());

  pitch_length_neutoral_ = new qt::DoubleSpinBox();
  pitch_length_neutoral_->setToolTip("The propeller pitch length when the pitch angle is 0°.");
  pitch_length_neutoral_->setDecimals(1);
  pitch_length_neutoral_->setMinimum(0.1);
  pitch_length_neutoral_->setValue(18);
  pitch_length_neutoral_->setSuffix(" inch");
  form->addRow(kPitchLengthNeutoralKey, pitch_length_neutoral_);
  form->labelForField(pitch_length_neutoral_)->setToolTip(pitch_length_neutoral_->toolTip());

  pitch_angle_limit_ = new DoubleRangeWidget();
  pitch_angle_limit_->setToolTip("Specify the range over which thrust varies linearly with propeller pitch angle.");
  pitch_angle_limit_->setDecimals(1);
  pitch_angle_limit_->setMinimum(-90);
  pitch_angle_limit_->setMaximum(+90);
  pitch_angle_limit_->setValue({ -10, 10 });
  pitch_angle_limit_->setSuffix(" deg");
  form->addRow(kPitchAngleLimitKey, pitch_angle_limit_);
  form->labelForField(pitch_angle_limit_)->setToolTip(pitch_angle_limit_->toolTip());

  center_pitch_angle_ = new qt::DoubleSpinBox();
  center_pitch_angle_->setDecimals(1);
  center_pitch_angle_->setMinimum(-90);
  center_pitch_angle_->setMaximum(+90);
  center_pitch_angle_->setValue(0);
  center_pitch_angle_->setSuffix(" deg");
  form->addRow(kCenterPitchAngleKey, center_pitch_angle_);
  form->labelForField(center_pitch_angle_)->setToolTip(center_pitch_angle_->toolTip());

  max_pitch_angle_rate_ = new qt::SpinBox();
  max_pitch_angle_rate_->setMinimum(0);
  max_pitch_angle_rate_->setValue(600);
  max_pitch_angle_rate_->setSuffix(" dps");
  form->addRow(kMaxPitchAngleRateKey, max_pitch_angle_rate_);
  form->labelForField(max_pitch_angle_rate_)->setToolTip(max_pitch_angle_rate_->toolTip());

  min_chord_ = new qt::SpinBox();
  min_chord_->setToolTip("Typically at the propeller tip.");
  min_chord_->setMinimum(1);
  min_chord_->setValue(45);
  min_chord_->setSuffix(" mm");
  form->addRow(kMinChordKey, min_chord_);
  form->labelForField(min_chord_)->setToolTip(min_chord_->toolTip());

  max_chord_ = new qt::SpinBox();
  max_chord_->setMinimum(1);
  max_chord_->setValue(60);
  max_chord_->setSuffix(" mm");
  form->addRow(kMaxChordKey, max_chord_);
  form->labelForField(max_chord_)->setToolTip(max_chord_->toolTip());
}

const char* PropellerWidget::name() const
{
  return "Propeller";
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

  if (min_chord_->value() > max_chord_->value()) {
    qt::qWarnBox(this, "Blade chord range is invalid.");
    return false;
  }

  return true;
}

void PropellerWidget::copyFrom(const BaseSelectedLinkSettingWidget* src)
{
  const auto derived = qt::qConstPointerCast<PropellerWidget>(src);

  num_blades_->setValue(derived->num_blades_->value());
  diameter_->setValue(derived->diameter_->value());
  pitch_length_neutoral_->setValue(derived->pitch_length_neutoral_->value());
  pitch_angle_limit_->setValue(derived->pitch_angle_limit_->value());
  center_pitch_angle_->setValue(derived->center_pitch_angle_->value());
  max_pitch_angle_rate_->setValue(derived->max_pitch_angle_rate_->value());
  min_chord_->setValue(derived->min_chord_->value());
  max_chord_->setValue(derived->max_chord_->value());
}

YAML::Node PropellerWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kNumBladesKey] = num_blades_->value();
  node[kDiameterKey] = diameter_->value();
  node[kPitchLengthNeutoralKey] = yaml::format(pitch_length_neutoral_->value());
  node[kPitchAngleLimitKey] = pitch_angle_limit_->value();
  node[kCenterPitchAngleKey] = yaml::format(center_pitch_angle_->value());
  node[kMaxPitchAngleRateKey] = max_pitch_angle_rate_->value();
  node[kMinChordKey] = min_chord_->value();
  node[kMaxChordKey] = max_chord_->value();

  return node;
}

void PropellerWidget::load(const YAML::Node& node)
{
  num_blades_->setValue(node[kNumBladesKey].as<int>());
  diameter_->setValue(node[kDiameterKey].as<int>());
  pitch_length_neutoral_->setValue(node[kPitchLengthNeutoralKey].as<double>());
  pitch_angle_limit_->setValue(node[kPitchAngleLimitKey].as<st::Range<double>>());
  center_pitch_angle_->setValue(node[kCenterPitchAngleKey].as<double>());
  max_pitch_angle_rate_->setValue(node[kMaxPitchAngleRateKey].as<int>());
  min_chord_->setValue(node[kMinChordKey].as<int>());
  max_chord_->setValue(node[kMaxChordKey].as<int>());
}

int PropellerWidget::numBlades() const
{
  return num_blades_->value();
}

double PropellerWidget::diameter() const
{
  return st::inch2meter(diameter_->value());
}

double PropellerWidget::radius() const
{
  return diameter() / 2;
}

double PropellerWidget::pitchLengthNeutoral() const
{
  return st::inch2meter(pitch_length_neutoral_->value());
}

double PropellerWidget::pitchAngleNeutoral() const
{
  return std::atan(pitchLengthNeutoral() / (M_PI * diameter()));
}

st::Range<double> PropellerWidget::pitchAngleLimit() const
{
  const auto lower = st::deg2rad(pitch_angle_limit_->value().lower);
  const auto upper = st::deg2rad(pitch_angle_limit_->value().upper);
  return { lower, upper };
}

double PropellerWidget::centerPitchAngle() const
{
  return st::deg2rad(center_pitch_angle_->value());
}

double PropellerWidget::maxPitchAngleRate() const
{
  return st::deg2rad(max_pitch_angle_rate_->value());
}

double PropellerWidget::minChord() const
{
  return min_chord_->value() * 1e-3;
}

double PropellerWidget::maxChord() const
{
  return max_chord_->value() * 1e-3;
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
