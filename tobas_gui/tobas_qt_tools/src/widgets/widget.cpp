#include "tobas_qt_tools/widgets/widget.hpp"

namespace qt
{
bool Widget::close()
{
  for (const auto child : findChildren<Widget*>())
    child->close();

  return super::close();
}
}  // namespace qt
