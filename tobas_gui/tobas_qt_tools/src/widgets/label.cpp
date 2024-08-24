#include "tobas_qt_tools/widgets/label.hpp"

namespace qt
{
FramedLable::FramedLable(const QString& text, QWidget* parent) : super(text, parent)
{
  setStyleSheet("QLabel { border: 1px solid black; background-color: white; }");
}
}  // namespace qt
