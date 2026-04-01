// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_kdl/tree_mass_holder.hpp>
#include <tobas_qt_tools/widgets/framed_label.hpp>
#include <tobas_qt_tools/widgets/scroll_area.hpp>

#include "./frame_type.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
class RobotPropertiesWidget : public qt::ScrollArea
{
  Q_OBJECT

  using self = RobotPropertiesWidget;
  using super = qt::ScrollArea;

public:
  explicit RobotPropertiesWidget(const kdl::Tree& tree);

  void updateInternalDataStructures();

  void setFrameType(const FrameType& type);

private:
  kdl::TreeMassHolder mass_holder_;

  qt::FramedLabel* frame_type_;
  qt::FramedLabel* mass_;
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
