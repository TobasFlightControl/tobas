#include "tobas_qt_tools/widgets/framed_label.hpp"

namespace qt
{
FramedLabel::FramedLabel(const QString& text, QWidget* parent) : super(text, parent)
{
  setStyleSheet("QLabel { border: 1px solid black; background-color: white; }");
  setAlignment(Qt::AlignRight | Qt::AlignVCenter);
}
}  // namespace qt
