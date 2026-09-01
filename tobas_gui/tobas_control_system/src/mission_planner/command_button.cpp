// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/mission_planner/command_button.hpp"

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace
{
constexpr int kMaxWidth = 100;
constexpr int kMinHeight = 35;
}  // namespace

CommandButton::CommandButton(const QString& text) : super(text)
{
  setMaximumWidth(kMaxWidth);
  setMinimumHeight(kMinHeight);
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
}
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
