// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QPushButton>

namespace tobas::gui::gcs
{
class RestartButton : public QPushButton
{
  Q_OBJECT

  using super = QPushButton;

public:
  explicit RestartButton(int _radius);
};
}  // namespace tobas::gui::gcs
