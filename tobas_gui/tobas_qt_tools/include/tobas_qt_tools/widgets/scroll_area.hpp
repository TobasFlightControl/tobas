#pragma once

#include <QScrollArea>

namespace qt
{
/**
 * ===== QScrollAreaとの違い =====
 * - デフォルトでスクロール可能
 * - 追加メソッド
 */
class ScrollArea : public QScrollArea
{
  Q_OBJECT

public:
  explicit ScrollArea();

  void setLayout(QLayout* layout);
};
}  // namespace qt
