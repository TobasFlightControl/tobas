// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/widget.hpp>

namespace tobas::gui::at
{
class BaseWidget : public qt::Widget
{
  Q_OBJECT

public:
  /** Title displayed at the top of the page. */
  virtual const char* title() const = 0;

  /** Initialize the widget without changing the robot structure. */
  virtual void reset() = 0;
};
}  // namespace tobas::gui::at
