#include "tobas_qt_tools/widgets/table_widget.hpp"

namespace qt
{
void TableWidget::removeAll()
{
  while (rowCount() > 0)
    removeRow(0);
}
}  // namespace qt
