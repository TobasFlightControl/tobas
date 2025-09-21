#pragma once

#include <QGridLayout>

namespace qt
{
/**
 * ===== QFormLayoutとの違い =====
 * - 追加メソッド
 */
class GridLayout : public QGridLayout
{
  Q_OBJECT

  using self = GridLayout;
  using super = QGridLayout;

public:
  using QGridLayout::QGridLayout;

  void clear();
};
}  // namespace qt
