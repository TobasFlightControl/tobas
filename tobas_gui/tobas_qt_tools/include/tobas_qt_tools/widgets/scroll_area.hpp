#pragma once

#include <QScrollArea>

namespace qt
{
/**
 * ===== QScrollArea との違い =====
 * - デフォルトでスクロール可能
 * - 追加メソッド
 */
class ScrollArea : public QScrollArea
{
  Q_OBJECT

  using super = QScrollArea;

public:
  explicit ScrollArea(QWidget* parent = nullptr);

  /* ウィジェットの中にレイアウトをセットする． */
  void setLayout(QLayout* layout);

  /* 背景を透明化する． */
  void setBackgroundTransparent();
};
}  // namespace qt
