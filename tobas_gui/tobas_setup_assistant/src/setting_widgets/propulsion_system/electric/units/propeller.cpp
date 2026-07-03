// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/propeller.hpp"

#include <QVBoxLayout>

#include <tobas_qt_tools/cast.hpp>
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
PropellerWidget::PropellerWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  num_blades_ = new ParamGetterWidget_SpinBox("Number of Blades", "");
  num_blades_->setMinimum(2);
  num_blades_->setMaximum(5);
  num_blades_->setValue(2);
  rows->addWidget(num_blades_);

  diameter_ = new ParamGetterWidget_SpinBox("Propeller Diameter", "");
  diameter_->setMinimum(1);
  diameter_->setValue(9);
  diameter_->setSuffix(" inch");
  rows->addWidget(diameter_);

  pitch_ = new ParamGetterWidget_DoubleSpinBox("Propeller Pitch", "");
  pitch_->setDecimals(1);
  pitch_->setMinimum(0.1);
  pitch_->setValue(4.5);
  pitch_->setSuffix(" inch");
  rows->addWidget(pitch_);

  min_chord_ = new ParamGetterWidget_SpinBox("Minimum Blade Chord Length", "Typically at the propeller tip.");
  min_chord_->setMinimum(1);
  min_chord_->setValue(10);
  min_chord_->setSuffix(" mm");
  rows->addWidget(min_chord_);

  max_chord_ = new ParamGetterWidget_SpinBox("Maximum Blade Chord Length", "");
  max_chord_->setMinimum(1);
  max_chord_->setValue(30);
  max_chord_->setSuffix(" mm");
  rows->addWidget(max_chord_);

  rows->addStretch();
}

const char* PropellerWidget::name() const
{
  return "Propeller";
}

bool PropellerWidget::isValid()
{
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
  pitch_->setValue(derived->pitch_->getValue());
  min_chord_->setValue(derived->min_chord_->getValue());
  max_chord_->setValue(derived->max_chord_->getValue());
}

YAML::Node PropellerWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[num_blades_->name()] = num_blades_->getValue();
  node[diameter_->name()] = diameter_->getValue();
  node[pitch_->name()] = yaml::format(pitch_->getValue());
  node[min_chord_->name()] = min_chord_->getValue();
  node[max_chord_->name()] = max_chord_->getValue();

  return node;
}

void PropellerWidget::load(const YAML::Node& node)
{
  num_blades_->setValue(node[num_blades_->name()].as<int>());
  diameter_->setValue(node[diameter_->name()].as<int>());
  pitch_->setValue(node[pitch_->name()].as<double>());
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

double PropellerWidget::pitchLength() const
{
  return st::inch2meter(pitch_->getValue());
}

double PropellerWidget::pitchAngle() const
{
  return std::atan(pitchLength() / (M_PI * diameter()));
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
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
