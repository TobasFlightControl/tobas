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

  using super = QScrollArea;

public:
  explicit ScrollArea(QWidget* parent = nullptr);

  void setLayout(QLayout* layout);
};
}  // namespace qt
