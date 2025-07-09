#pragma once

#include <QTabWidget>

namespace qt
{
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

  /* 全てのタブを削除してメモリを開放する． */
  void removeAllTabs();
};
}  // namespace qt
