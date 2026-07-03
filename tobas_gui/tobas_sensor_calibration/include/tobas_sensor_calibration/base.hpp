// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QLabel>
#include <QVBoxLayout>

#include <tobas_qt_tools/widgets/widget.hpp>

namespace tobas
{
namespace gui
{
namespace sc
{
class BaseWidget : public qt::Widget
{
  Q_OBJECT

public:
  explicit BaseWidget();

  /* Title displayed at the top of the page. */
  virtual const char* title() const = 0;

  /* Initialize the widget without changing the robot structure. */
  virtual void reset() = 0;

protected:
  QVBoxLayout* rows_;

private:
  QLabel* title_;

private Q_SLOTS:
  void initialize();
};
}  // namespace sc
}  // namespace gui
}  // namespace tobas
