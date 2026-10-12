// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./tab_widget.hpp"

namespace tobas
{
namespace qt
{
class VerticalTabBar : public TabBar
{
  Q_OBJECT

  using super = TabBar;

protected:
  QSize tabSizeHint(int _index) const override;
  void paintEvent(QPaintEvent* _event) override;
};

class VerticalTabWidget : public TabWidget
{
  Q_OBJECT

  using super = TabWidget;

public:
  explicit VerticalTabWidget(QWidget* _parent = nullptr);

  void setTabBackgroundColor(int _index, const QColor& _color) override;
  void clearTabBackgroundColor(int _index) override;

private:
  VerticalTabBar* tab_bar_;
};
}  // namespace qt
}  // namespace tobas
