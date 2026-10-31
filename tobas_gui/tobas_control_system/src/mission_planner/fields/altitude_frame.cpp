// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/mission_planner/fields/altitude_frame.hpp"

#include <cstring>
#include <format>
#include <stdexcept>

#include <QHBoxLayout>
#include <magic_enum/magic_enum.hpp>

namespace tobas::gui::ctrl::field
{
namespace
{
constexpr char kRelativeToLaunchLabel[] = "Relative to Launch";
constexpr char kMeanSeaLevelLabel[] = "Mean Sea Level";

const char* altFrameToText(mission::AltitudeFrame _frame)
{
  switch (_frame) {
    case mission::kRelativeToLaunch:
      return kRelativeToLaunchLabel;
    case mission::kMeanSeaLevel:
      return kMeanSeaLevelLabel;
    default:
      throw std::runtime_error(std::format("Invalid altitude frame: {}", (int)_frame));
  }
}

mission::AltitudeFrame textToAltFrame(const char* _text)
{
  if (std::strcmp(_text, kRelativeToLaunchLabel) == 0) {
    return mission::kRelativeToLaunch;
  }
  else if (std::strcmp(_text, kMeanSeaLevelLabel) == 0) {
    return mission::kMeanSeaLevel;
  }
  else {
    throw std::runtime_error(std::format("Invalid altitude frame text: {}", _text));
  }
}
}  // namespace

AltitudeFrameWidget::AltitudeFrameWidget()
{
  combobox_ = new qt::ComboBox();
  for (const auto alt_frame : magic_enum::enum_values<mission::AltitudeFrame>()) {
    combobox_->addItem(altFrameToText(alt_frame));
  }

  const auto cols = new QHBoxLayout(this);
  cols->addWidget(combobox_);

  connect(combobox_, qOverload<int>(&QComboBox::currentIndexChanged), this, &BaseFieldWidget::updated);
}

const char* AltitudeFrameWidget::label() const
{
  return "Altitude Frame";
}

mission::AltitudeFrame AltitudeFrameWidget::getValue() const
{
  return textToAltFrame(combobox_->currentText().toUtf8().constData());
}

void AltitudeFrameWidget::setValue(mission::AltitudeFrame _value)
{
  combobox_->setCurrentText(altFrameToText(_value));
}
}  // namespace tobas::gui::ctrl::field
