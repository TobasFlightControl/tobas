#pragma once

#include <QWidget>

namespace qt
{
/**
 * ===== QWidgetとの違い =====
 * - closeで子ウィジェットのcloseを再帰的に呼び出す
 * - 追加メソッド
 */
class Widget : public QWidget
{
  Q_OBJECT

  using super = QWidget;

public:
  using super::QWidget;

  QPoint getCenter() const;

public Q_SLOTS:
  bool close();
};
}  // namespace qt
