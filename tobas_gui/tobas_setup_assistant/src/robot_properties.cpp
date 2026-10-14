// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/robot_properties.hpp"

#include <tobas_qt_tools/layouts/form_layout.hpp>

namespace tobas::gui::sa
{
RobotPropertiesWidget::RobotPropertiesWidget(const uadf::Model& _uadf, const kdl::Tree& _tree)
  : uadf_(_uadf), tree_(_tree), mass_holder_(_tree)
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

  mass_holder_.updateInternalDataStructures();
  mass_->setText(QString::number(mass_holder_.getMass()) + " kg");
}

void RobotPropertiesWidget::setFrameType(FrameType _type)
{
  const auto text = QString::fromStdString(textFromEnum(_type));
  frame_type_->setText(text);
}
}  // namespace tobas::gui::sa
