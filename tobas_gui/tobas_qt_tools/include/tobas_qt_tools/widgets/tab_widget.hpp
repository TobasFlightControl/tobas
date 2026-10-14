// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QTabBar>
#include <QTabWidget>
#include <QWheelEvent>

namespace tobas::qt
{
class TabBar : public QTabBar
{
  Q_OBJECT

  using super = QTabBar;

public:
  explicit TabBar();

  void enableWheelEvent(bool _enable);

  void setTabBackgroundColor(int _index, const QColor& _color);
  void clearTabBackgroundColor(int _index);

protected:
  QMap<int, QColor> colors_;

  void wheelEvent(QWheelEvent* _event) override;
  void paintEvent(QPaintEvent* _event) override;

private:
  bool enable_wheel_event_ = true;
};

/**
 * ===== Differences from `QTabWidget` =====
 * - Additional methods
 */
class TabWidget : public QTabWidget
{
  Q_OBJECT

  using super = QTabWidget;

public:
  explicit TabWidget(QWidget* _parent = nullptr);

  /**
   * Ignore mouse wheel events.
   *
   * @note Must be called before TabBar settings such as `setMovable`.
   */
  virtual void enableWheelEvent(bool _enable);

  virtual void setTabBackgroundColor(int _index, const QColor& _color);
  virtual void clearTabBackgroundColor(int _index);

  void setTabEnabled(QWidget* _tab, bool _enabled);
  void setTabVisible(QWidget* _tab, bool _visible);

  void setTabWidth(int _width);
  void setTabHeight(int _height);
  void setTabSize(int _width, int _height);

  /** Delete all tabs and free memory. */
  void removeAllTabs();

private:
  TabBar* tab_bar_;
};
}  // namespace tobas::qt
