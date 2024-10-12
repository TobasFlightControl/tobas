#pragma once

#include <QLabel>

namespace qt
{
class FramedLabel : public QLabel
{
  using super = QLabel;

public:
  explicit FramedLabel(const QString& text = "", QWidget* parent = nullptr);
};
}  // namespace qt
