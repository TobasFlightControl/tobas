#pragma once

#include "./tab_widget.hpp"

namespace qt
{
class VerticalTabBar : public TabBar
{
  Q_OBJECT

  using super = TabBar;

protected:
  QSize tabSizeHint(int index) const override;
  void paintEvent(QPaintEvent* event) override;
};

class VerticalTabWidget : public TabWidget
{
  Q_OBJECT

  using super = TabWidget;

public:
  explicit VerticalTabWidget(QWidget* parent = nullptr);
};
}  // namespace qt
