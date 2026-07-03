// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QLabel>
#include <QVBoxLayout>

namespace tobas
{
namespace gui
{
namespace bm
{
class BaseConfigWidget : public QWidget
{
  Q_OBJECT

public:
  explicit BaseConfigWidget();

  /* Title displayed at the top of the page. */
  virtual const char* title() const = 0;

  /* Initialize the configuration. */
  virtual void reset() = 0;

  /* Method executed when a boot device is connected. */
  virtual bool onConnected() = 0;

protected:
  QVBoxLayout* rows_;

private:
  QLabel* title_;

private Q_SLOTS:
  void initialize();
};
}  // namespace bm
}  // namespace gui
}  // namespace tobas
