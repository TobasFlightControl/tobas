// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QWidget>

namespace tobas
{
namespace gui
{
namespace log
{
class BasePlotWidget : public QWidget
{
  Q_OBJECT

public:
  virtual void clear() = 0;
  virtual void setTimeScale(double t_start, double t_stop) = 0;
};
}  // namespace log
}  // namespace gui
}  // namespace tobas
