// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_kdl/tree_mass_holder.hpp>
#include <tobas_qt_tools/widgets/framed_label.hpp>
#include <tobas_qt_tools/widgets/scroll_area.hpp>
#include <tobas_uadf/model.hpp>

#include "./frame_type.hpp"

namespace tobas::gui::sa
{
class RobotPropertiesWidget : public qt::ScrollArea
{
  Q_OBJECT

public:
  explicit RobotPropertiesWidget(const uadf::Model& _uadf, const kdl::Tree& _tree);

  void updateInternalDataStructures();

  void setFrameType(FrameType _type);

private:
  const uadf::Model& uadf_;
  const kdl::Tree& tree_;

  kdl::TreeMassHolder mass_holder_;

  qt::FramedLabel* robot_name_;
  qt::FramedLabel* frame_type_;
  qt::FramedLabel* mass_;
};
}  // namespace tobas::gui::sa
