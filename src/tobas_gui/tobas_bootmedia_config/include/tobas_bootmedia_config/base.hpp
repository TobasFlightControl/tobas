// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QWidget>

namespace tobas::gui::bm
{
class BaseConfigWidget : public QWidget
{
  Q_OBJECT

public:
  explicit BaseConfigWidget();

  /** Title displayed at the top of the page. */
  virtual const char* title() const = 0;

  /** Initialize the configuration. */
  virtual void reset() = 0;

  /** Method executed when a boot device is connected. */
  virtual bool onConnected() = 0;
};
}  // namespace tobas::gui::bm
