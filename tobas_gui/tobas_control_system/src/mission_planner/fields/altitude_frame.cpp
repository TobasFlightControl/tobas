// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/mission_planner/fields/altitude_frame.hpp"

#include <string.h>

#include <format>
#include <stdexcept>

#include <QHBoxLayout>
#include <magic_enum/magic_enum.hpp>

#define MEAN_SEA_LEVEL_LABEL "Mean Sea Level"
#define RELATIVE_TO_LAUNCH_LABEL "Relative to Launch"

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace field
{
namespace
{
const char* altFrameToText(mission::AltitudeFrame frame)
{
  switch (frame) {
    case mission::kMeanSeaLevel:
      return MEAN_SEA_LEVEL_LABEL;
    case mission::kRelativeToLaunch:
      return RELATIVE_TO_LAUNCH_LABEL;
    default:
      throw std::runtime_error(std::format("Invalid altitude frame: {}", (int)frame));
  }
}

mission::AltitudeFrame textToAltFrame(const char* text)
{
  if (strcmp(text, MEAN_SEA_LEVEL_LABEL) == 0) {
    return mission::kMeanSeaLevel;
  }
  else if (strcmp(text, RELATIVE_TO_LAUNCH_LABEL) == 0) {
    return mission::kRelativeToLaunch;
  }
  else {
    throw std::runtime_error(std::format("Invalid altitude frame text: {}", text));
  }
}
}  // namespace

AltitudeFrameWidget::AltitudeFrameWidget()
{
  combobox_ = new qt::ComboBox();
  combobox_->addItem(altFrameToText(mission::kRelativeToLaunch));  // TODO: 他の選択肢も選べるようにする
  // for (const auto alt_frame : magic_enum::enum_values<mission::AltitudeFrame>())
  //   combobox_->addItem(altFrameToText(alt_frame));

  const auto cols = new QHBoxLayout();
  setLayout(cols);
  cols->addWidget(combobox_);

  connect(combobox_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &BaseFieldWidget::updated);
}

const char* AltitudeFrameWidget::label() const
{
  return "Altitude Frame";
}

mission::AltitudeFrame AltitudeFrameWidget::getValue() const
{
  return textToAltFrame(combobox_->currentText().toUtf8());
}

void AltitudeFrameWidget::setValue(mission::AltitudeFrame value)
{
  combobox_->setCurrentText(altFrameToText(value));
}
}  // namespace field
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
