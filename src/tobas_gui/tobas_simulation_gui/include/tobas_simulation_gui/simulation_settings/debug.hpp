// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QCheckBox>

namespace tobas::gui::sim
{
class DebugWidget : public QWidget
{
  Q_OBJECT

public:
  explicit DebugWidget();

  bool userDebug() const;

private:
  QCheckBox* user_debug_;
};
}  // namespace tobas::gui::sim
