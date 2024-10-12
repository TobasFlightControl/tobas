#pragma once

#include <QLabel>

namespace qt
{
/**
 * ===== QLabelとの違い =====
 * - コンストラクタでフォントを指定可
 */
class Label : public QLabel
{
  using super = QLabel;

public:
  explicit Label(
    const QString& text = "",
    int point_size = -1,
    int weight = -1,
    bool italic = false,
    QWidget* parent = nullptr);
};
}  // namespace qt
