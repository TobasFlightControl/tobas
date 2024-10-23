#pragma once

#include <QStackedWidget>

namespace qt
{
/**
 * ===== QStackedWidgetとの違い =====
 * - setCurrentIndexを安定化
 * - 追加メソッド
 */
class StackedWidget : public QStackedWidget
{
  Q_OBJECT

  using super = QStackedWidget;

public:
  using super::QStackedWidget;

  /* 全てのウィジェットを削除し，メモリを開放する． */
  void clear();

public Q_SLOTS:
  void setCurrentIndex(int index);
};
}  // namespace qt
