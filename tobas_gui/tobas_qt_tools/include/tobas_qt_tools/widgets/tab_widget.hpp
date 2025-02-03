#pragma once

#include <QTabWidget>
#include <QTabBar>

namespace qt
{
class TabBarWithNoWheelEvent : public QTabBar
{
  Q_OBJECT

  using super = QTabBar;

public:
  using super::QTabBar;

  void wheelEvent(QWheelEvent* event) override;
};

/**
 * ===== QtabWidgetとの違い =====
 * - イテレータを定義
 * - 追加メソッド
 */
class TabWidget : public QTabWidget
{
  Q_OBJECT

  using super = QTabWidget;

public:
  using super::QTabWidget;

  /**
   * @brief マウスホイールイベントを無視する．
   * @note setMovableなどのTabBarの設定の前に呼ぶ必要がある．
   */
  virtual void ignoreWheelEvent();

  void switchTab(QWidget* tab);

  void setTabWidth(int width);
  void setTabHeight(int height);
  void setTabSize(int width, int height);
};
}  // namespace qt
