// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QTabBar>
#include <QTabWidget>
#include <QWheelEvent>

namespace tobas
{
namespace qt
{
class TabBar : public QTabBar
{
  Q_OBJECT

  using super = QTabBar;

public:
  explicit TabBar();

  void enableWheelEvent(bool enable);

  void setTabBackgroundColor(int index, const QColor& color);
  void clearTabBackgroundColor(int index);

protected:
  QMap<int, QColor> colors_;

  void wheelEvent(QWheelEvent* event) override;
  void paintEvent(QPaintEvent* event) override;

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
  explicit TabWidget(QWidget* parent = nullptr);

  /**
   * @brief Ignore mouse wheel events.
   * @note Must be called before TabBar settings such as `setMovable`.
   */
  virtual void enableWheelEvent(bool enable);

  virtual void setTabBackgroundColor(int index, const QColor& color);
  virtual void clearTabBackgroundColor(int index);

  void setTabEnabled(QWidget* tab, bool enabled);
  void setTabVisible(QWidget* tab, bool visible);

  void setTabWidth(int width);
  void setTabHeight(int height);
  void setTabSize(int width, int height);

  /* Delete all tabs and free memory. */
  void removeAllTabs();

private:
  TabBar* tab_bar_;
};
}  // namespace qt
}  // namespace tobas
