#pragma once

#include <QTableWidget>

namespace qt
{
/**
 * ===== TableWidgetとの違い =====
 * - 追加メソッド
 */
class TableWidget : public QTableWidget
{
  Q_OBJECT

  using super = QTableWidget;

public:
  using super::QTableWidget;

  /* 全ての行を削除する．clearとは異なり，内容に加えセルまで削除する． */
  void removeAll();
};
}  // namespace qt
