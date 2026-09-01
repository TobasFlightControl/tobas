// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QPushButton>

namespace tobas
{
namespace gui
{
namespace ctrl
{
class CommandButton : public QPushButton
{
  Q_OBJECT

  using self = CommandButton;
  using super = QPushButton;

public:
  explicit CommandButton(const QString& text);
};
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
