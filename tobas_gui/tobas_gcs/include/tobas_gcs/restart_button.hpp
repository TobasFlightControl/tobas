// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QPushButton>

namespace tobas
{
namespace gui
{
namespace gcs
{
class RestartButton : public QPushButton
{
  Q_OBJECT

  using self = RestartButton;
  using super = QPushButton;

public:
  explicit RestartButton(int radius);
};
}  // namespace gcs
}  // namespace gui
}  // namespace tobas
