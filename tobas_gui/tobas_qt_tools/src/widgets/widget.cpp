#include "tobas_qt_tools/widgets/widget.hpp"

namespace qt
{
bool Widget::close()
{
  for (const auto child : findChildren<Widget*>())
    child->close();

  return super::close();
}

QPoint Widget::getCenter() const
{
  return QPoint(width() / 2, height() / 2);
}
}  // namespace qt
