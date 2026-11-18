// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/propeller.hpp"

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_yaml_tools/format.hpp>

namespace tobas::gui::sa::propulsion::electric
{
namespace
{
constexpr char kNumBladesLabel[] = "Number of Blades";
constexpr char kDiameterLabel[] = "Propeller Diameter";
constexpr char kPitchLabel[] = "Propeller Pitch";
constexpr char kMinChordLabel[] = "Minimum Blade Chord Length";
constexpr char kMaxChordLabel[] = "Maximum Blade Chord Length";
}  // namespace

PropellerWidget::PropellerWidget()
{
  const auto form = new qt::FormLayout();
  setLayout(form);  // Do not pass `this` to the layout constructor.

  num_blades_ = new qt::SpinBox();
  num_blades_->setMinimum(2);
  num_blades_->setMaximum(5);
  form->addRow(kNumBladesLabel, num_blades_);

  diameter_ = new qt::SpinBox();
  diameter_->setMinimum(1);
  diameter_->setSuffix(" inch");
  form->addRow(kDiameterLabel, diameter_);

  pitch_ = new qt::DoubleSpinBox();
  pitch_->setDecimals(1);
  pitch_->setMinimum(0.1);
  pitch_->setSuffix(" inch");
  form->addRow(kPitchLabel, pitch_);

  min_chord_ = new qt::SpinBox();
  min_chord_->setToolTip("Typically at the propeller tip.");
  min_chord_->setMinimum(1);
  min_chord_->setSuffix(" mm");
  form->addRow(kMinChordLabel, min_chord_);
  form->labelForField(min_chord_)->setToolTip(min_chord_->toolTip());

  max_chord_ = new qt::SpinBox();
  max_chord_->setMinimum(1);
  max_chord_->setSuffix(" mm");
  form->addRow(kMaxChordLabel, max_chord_);
}

const char* PropellerWidget::name() const
{
  return "Propeller";
}

void PropellerWidget::setToDefaults()
{
  num_blades_->setValue(2);
  diameter_->setValue(9);
  pitch_->setValue(4.5);
  min_chord_->setValue(10);
  max_chord_->setValue(30);
}

bool PropellerWidget::isValid()
{
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
  pitch_->setValue(derived->pitch_->value());
  min_chord_->setValue(derived->min_chord_->value());
  max_chord_->setValue(derived->max_chord_->value());
}

YAML::Node PropellerWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kNumBladesLabel] = num_blades_->value();
  node[kDiameterLabel] = diameter_->value();
  node[kPitchLabel] = yaml::format(pitch_->value());
  node[kMinChordLabel] = min_chord_->value();
  node[kMaxChordLabel] = max_chord_->value();

  return node;
}

void PropellerWidget::load(const YAML::Node& _node)
{
  num_blades_->setValue(_node[kNumBladesLabel].as<int>());
  diameter_->setValue(_node[kDiameterLabel].as<int>());
  pitch_->setValue(_node[kPitchLabel].as<double>());
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

double PropellerWidget::pitchLength() const
{
  return st::inch2meter(pitch_->value());
}

double PropellerWidget::pitchAngle() const
{
  return std::atan(pitchLength() / (M_PI * diameter()));
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
}  // namespace tobas::gui::sa::propulsion::electric
