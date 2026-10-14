// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QPushButton>

namespace tobas::gui::ctrl
{
class CommandButton : public QPushButton
{
  Q_OBJECT

  using super = QPushButton;

public:
  explicit CommandButton(const QString& _text);
};
}  // namespace tobas::gui::ctrl
