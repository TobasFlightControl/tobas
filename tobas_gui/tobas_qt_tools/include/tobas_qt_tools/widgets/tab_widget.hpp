#pragma once

#include <QTabBar>
#include <QTabWidget>

namespace qt
{
class TabBar : public QTabBar
{
  Q_OBJECT

  using super = QTabBar;

public:
  void enableWheelEvent(bool enable);

protected:
  void wheelEvent(QWheelEvent* event) override;

private:
  bool enable_wheel_event_ = true;
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
  explicit TabWidget(QWidget* parent = nullptr);

  /**
   * @brief マウスホイールイベントを無視する．
   * @note setMovableなどのTabBarの設定の前に呼ぶ必要がある．
   */
  virtual void enableWheelEvent(bool enable);

  void setTabVisible(QWidget* tab, bool visible);

  void setTabWidth(int width);
  void setTabHeight(int height);
  void setTabSize(int width, int height);
};
}  // namespace qt
