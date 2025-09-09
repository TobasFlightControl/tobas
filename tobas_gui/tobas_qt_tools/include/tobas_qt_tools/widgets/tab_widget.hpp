#pragma once

#include <QTabBar>
#include <QTabWidget>
#include <QWheelEvent>

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

  virtual void setTabBackgroundColor(int index, const QColor& color);
  virtual void clearTabBackgroundColor(int index);

  void setTabEnabled(QWidget* tab, bool enabled);
  void setTabVisible(QWidget* tab, bool visible);

  void setTabWidth(int width);
  void setTabHeight(int height);
  void setTabSize(int width, int height);

  /* 全てのタブを削除してメモリを開放する． */
  void removeAllTabs();

private:
  TabBar* tab_bar_;
};
}  // namespace qt
