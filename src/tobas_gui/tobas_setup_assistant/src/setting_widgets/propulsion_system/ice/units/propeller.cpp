// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/propulsion_units/propeller.hpp"

#include <QFormLayout>

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_yaml_tools/convert/range.hpp>
#include <tobas_yaml_tools/format.hpp>

namespace tobas::gui::sa::propulsion::ice
{
namespace
{
constexpr char kNumBladesLabel[] = "Number of Blades";
constexpr char kDiameterLabel[] = "Propeller Diameter";
constexpr char kPitchLengthNeutoralLabel[] = "Neutral Propeller Pitch";
constexpr char kPitchAngleLimitLabel[] = "Pitch Angle Limit";
constexpr char kCenterPitchAngleLabel[] = "Center Pitch Angle";
constexpr char kMaxPitchAngleRateLabel[] = "Maximum Blade Pitch Rate";
constexpr char kMinChordLabel[] = "Minimum Blade Chord Length";
constexpr char kMaxChordLabel[] = "Maximum Blade Chord Length";
}  // namespace

PropellerWidget::PropellerWidget()
{
  const auto form = new QFormLayout();
  setLayout(form);  // Do not pass `this` to the layout constructor.

  num_blades_ = new qt::SpinBox();
  num_blades_->setToolTip("Number of blades on one propeller.");
  num_blades_->setMinimum(2);
  num_blades_->setMaximum(5);
  form->addRow(kNumBladesLabel, num_blades_);
  form->labelForField(num_blades_)->setToolTip(num_blades_->toolTip());

  diameter_ = new qt::SpinBox();
  diameter_->setToolTip("Diameter of the circle swept by the propeller tips, in inches.");
  diameter_->setMinimum(1);
  diameter_->setSuffix(" inch");
  form->addRow(kDiameterLabel, diameter_);
  form->labelForField(diameter_)->setToolTip(diameter_->toolTip());

  pitch_length_neutoral_ = new qt::DoubleSpinBox();
  pitch_length_neutoral_->setToolTip(
    "Geometric pitch length in inches at the zero position of the variable-pitch mechanism. "
    "This reference position does not imply zero thrust.");
  pitch_length_neutoral_->setDecimals(1);
  pitch_length_neutoral_->setMinimum(0.1);
  pitch_length_neutoral_->setSuffix(" inch");
  form->addRow(kPitchLengthNeutoralLabel, pitch_length_neutoral_);
  form->labelForField(pitch_length_neutoral_)->setToolTip(pitch_length_neutoral_->toolTip());

  pitch_angle_limit_ = new DoubleRangeWidget();
  pitch_angle_limit_->setToolTip(
    "Minimum and maximum variable-pitch angles in degrees relative to the zero position. "
    "Select the range where the propeller model is valid; the range must include negative and positive angles.");
  pitch_angle_limit_->setDecimals(1);
  pitch_angle_limit_->setRange(-90, 90);
  pitch_angle_limit_->setToolTip(
    "Minimum and maximum variable-pitch angles in degrees relative to the zero position. "
    "Select the range where the propeller model is valid; the range must include negative and positive angles.");
  pitch_angle_limit_->setSuffix(" deg");
  form->addRow(kPitchAngleLimitLabel, pitch_angle_limit_);
  form->labelForField(pitch_angle_limit_)->setToolTip(pitch_angle_limit_->toolTip());

  center_pitch_angle_ = new qt::DoubleSpinBox();
  center_pitch_angle_->setToolTip(
    "Reference propeller pitch angle in degrees used by the control model. Must lie within the pitch angle limits.");
  center_pitch_angle_->setDecimals(1);
  center_pitch_angle_->setMinimum(-90);
  center_pitch_angle_->setMaximum(+90);
  center_pitch_angle_->setSuffix(" deg");
  form->addRow(kCenterPitchAngleLabel, center_pitch_angle_);
  form->labelForField(center_pitch_angle_)->setToolTip(center_pitch_angle_->toolTip());

  max_pitch_angle_rate_ = new qt::SpinBox();
  max_pitch_angle_rate_->setToolTip(
    "Maximum rate of change of propeller pitch in degrees per second, used by the simulation actuator model.");
  max_pitch_angle_rate_->setMinimum(0);
  max_pitch_angle_rate_->setSuffix(" deg/s");
  form->addRow(kMaxPitchAngleRateLabel, max_pitch_angle_rate_);
  form->labelForField(max_pitch_angle_rate_)->setToolTip(max_pitch_angle_rate_->toolTip());

  min_chord_ = new qt::SpinBox();
  min_chord_->setToolTip(
    "Minimum blade chord length in millimeters. "
    "The mean of the minimum and maximum chord lengths is used by the blade model.");
  min_chord_->setMinimum(1);
  min_chord_->setSuffix(" mm");
  form->addRow(kMinChordLabel, min_chord_);
  form->labelForField(min_chord_)->setToolTip(min_chord_->toolTip());

  max_chord_ = new qt::SpinBox();
  max_chord_->setToolTip("Maximum blade chord length in millimeters. Must be at least the minimum chord length.");
  max_chord_->setMinimum(1);
  max_chord_->setSuffix(" mm");
  form->addRow(kMaxChordLabel, max_chord_);
  form->labelForField(max_chord_)->setToolTip(max_chord_->toolTip());
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
    qt::qWarnBox(this, "The center pitch angle is outside the allowed range.");
    return false;
  }

  if (min_chord_->value() > max_chord_->value()) {
    qt::qWarnBox(this, "Blade chord range is invalid.");
    return false;
  }

  return true;
}

void PropellerWidget::copyFrom(const BaseSelectedLinkSettingWidget* _src)
{
  const auto derived = qt::qConstPointerCast<PropellerWidget>(_src);

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

  node[kNumBladesLabel] = num_blades_->value();
  node[kDiameterLabel] = diameter_->value();
  node[kPitchLengthNeutoralLabel] = yaml::format(pitch_length_neutoral_->value());
  node[kPitchAngleLimitLabel] = pitch_angle_limit_->value();
  node[kCenterPitchAngleLabel] = yaml::format(center_pitch_angle_->value());
  node[kMaxPitchAngleRateLabel] = max_pitch_angle_rate_->value();
  node[kMinChordLabel] = min_chord_->value();
  node[kMaxChordLabel] = max_chord_->value();

  return node;
}

void PropellerWidget::load(const YAML::Node& _node)
{
  num_blades_->setValue(_node[kNumBladesLabel].as<int>());
  diameter_->setValue(_node[kDiameterLabel].as<int>());
  pitch_length_neutoral_->setValue(_node[kPitchLengthNeutoralLabel].as<double>());
  pitch_angle_limit_->setValue(_node[kPitchAngleLimitLabel].as<st::Range<double>>());
  center_pitch_angle_->setValue(_node[kCenterPitchAngleLabel].as<double>());
  max_pitch_angle_rate_->setValue(_node[kMaxPitchAngleRateLabel].as<int>());
  min_chord_->setValue(_node[kMinChordLabel].as<int>());
  max_chord_->setValue(_node[kMaxChordLabel].as<int>());
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
  const auto limit = pitch_angle_limit_->value();
  const auto lower = st::deg2rad(limit.lower);
  const auto upper = st::deg2rad(limit.upper);
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
}  // namespace tobas::gui::sa::propulsion::ice
