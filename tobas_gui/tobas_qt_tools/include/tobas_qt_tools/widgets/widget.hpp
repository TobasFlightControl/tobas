#pragma once

#include <QWidget>

namespace qt
{
/**
 * ===== QWidgetとの違い =====
 * - closeで子ウィジェットのcloseを再帰的に呼び出す
 */
class Widget : public QWidget
{
  Q_OBJECT

  using super = QWidget;

public:
  using super::QWidget;

public Q_SLOTS:
  bool close();
};
}  // namespace qt
