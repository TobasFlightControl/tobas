// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QCheckBox>

namespace tobas
{
namespace gui
{
namespace sim
{
class DebugWidget : public QWidget
{
  Q_OBJECT

  using self = DebugWidget;
  using super = QWidget;

public:
  explicit DebugWidget();

  bool userDebug() const;

private:
  QCheckBox* user_debug_;
};
}  // namespace sim
}  // namespace gui
}  // namespace tobas
