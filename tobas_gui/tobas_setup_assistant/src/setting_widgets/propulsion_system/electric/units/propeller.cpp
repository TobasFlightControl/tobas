// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/propeller.hpp"

#include <QVBoxLayout>

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_yaml_tools/format.hpp>

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
namespace
{
// Keep the existing serialized keys independent of the form labels.
constexpr char kNumBladesKey[] = "Number of Blades";
constexpr char kDiameterKey[] = "Propeller Diameter";
constexpr char kPitchKey[] = "Propeller Pitch";
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
  diameter_->setValue(9);
  diameter_->setSuffix(" inch");
  form->addRow(kDiameterKey, diameter_);
  form->labelForField(diameter_)->setToolTip(diameter_->toolTip());

  pitch_ = new qt::DoubleSpinBox();
  pitch_->setDecimals(1);
  pitch_->setMinimum(0.1);
  pitch_->setValue(4.5);
  pitch_->setSuffix(" inch");
  form->addRow(kPitchKey, pitch_);
  form->labelForField(pitch_)->setToolTip(pitch_->toolTip());

  min_chord_ = new qt::SpinBox();
  min_chord_->setToolTip("Typically at the propeller tip.");
  min_chord_->setMinimum(1);
  min_chord_->setValue(10);
  min_chord_->setSuffix(" mm");
  form->addRow(kMinChordKey, min_chord_);
  form->labelForField(min_chord_)->setToolTip(min_chord_->toolTip());

  max_chord_ = new qt::SpinBox();
  max_chord_->setMinimum(1);
  max_chord_->setValue(30);
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
  pitch_->setValue(derived->pitch_->value());
  min_chord_->setValue(derived->min_chord_->value());
  max_chord_->setValue(derived->max_chord_->value());
}

YAML::Node PropellerWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kNumBladesKey] = num_blades_->value();
  node[kDiameterKey] = diameter_->value();
  node[kPitchKey] = yaml::format(pitch_->value());
  node[kMinChordKey] = min_chord_->value();
  node[kMaxChordKey] = max_chord_->value();

  return node;
}

void PropellerWidget::load(const YAML::Node& node)
{
  num_blades_->setValue(node[kNumBladesKey].as<int>());
  diameter_->setValue(node[kDiameterKey].as<int>());
  pitch_->setValue(node[kPitchKey].as<double>());
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
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
