// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/robot_properties.hpp"

#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_std_tools/check.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
RobotPropertiesWidget::RobotPropertiesWidget(const uadf::Model& uadf, const kdl::Tree& tree)
  : uadf_(uadf), tree_(tree), mass_holder_(tree)
{
  robot_name_ = new qt::FramedLabel();
  frame_type_ = new qt::FramedLabel();
  mass_ = new qt::FramedLabel();

  const auto form = new qt::FormLayout();
  form->setHorizontalSpacing(30);
  setLayout(form);
  form->addRow("Robot Name", robot_name_);
  form->addRow("Frame Type", frame_type_);
  form->addRow("Total Mass", mass_);
}

void RobotPropertiesWidget::updateInternalDataStructures()
{
  robot_name_->setText(QString::fromStdString(uadf_.urdf->getName()));

  TOBAS_CHECK(mass_holder_.updateInternalDataStructures());
  mass_->setText(QString::number(mass_holder_.getMass()) + " kg");
}

void RobotPropertiesWidget::setFrameType(const FrameType& type)
{
  const auto text = QString::fromStdString(textFromEnum(type));
  frame_type_->setText(text);
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas
