// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/mission_planner/command_button.hpp"

namespace tobas
{
namespace gui
{
namespace ctrl
{
CommandButton::CommandButton(const QString& text) : super(text)
{
  setMaximumWidth(kMaxWidth);
  setFixedHeight(kFixedHeight);
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
}
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
