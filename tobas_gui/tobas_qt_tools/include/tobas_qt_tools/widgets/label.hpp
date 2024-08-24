#pragma once

#include <QLabel>

namespace qt
{
class FramedLable : public QLabel
{
  using super = QLabel;

public:
  explicit FramedLable(const QString& text = "", QWidget* parent = nullptr);
};
}  // namespace qt
