#pragma once

#include <QLabel>

namespace qt
{
/**
 * @brief 黒枠付きのラベル．
 * QLineEditをReadOnly + NoFocusにしたものに近いが，こちらの方が効率的．
 */
class FramedLabel : public QLabel
{
  using super = QLabel;

public:
  explicit FramedLabel(const QString& text = "", QWidget* parent = nullptr);
};
}  // namespace qt
