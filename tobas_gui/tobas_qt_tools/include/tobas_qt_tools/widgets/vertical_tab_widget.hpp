#pragma once

#include "./tab_widget.hpp"

namespace qt
{
class VerticalTabBar : public QTabBar
{
  Q_OBJECT

  using super = QTabBar;

public:
  using super::QTabBar;

protected:
  QSize tabSizeHint(int index) const override;
  void paintEvent(QPaintEvent* event) override;
};

class VerticalTabBarWithNoWheelEvent : public QTabBar
{
  Q_OBJECT

  using super = QTabBar;

public:
  using super::QTabBar;

protected:
  void wheelEvent(QWheelEvent* event) override;
};

class VerticalTabWidget : public TabWidget
{
  Q_OBJECT

  using super = TabWidget;

public:
  explicit VerticalTabWidget(QWidget* parent = nullptr);

  void ignoreWheelEvent() override;
};
}  // namespace qt
