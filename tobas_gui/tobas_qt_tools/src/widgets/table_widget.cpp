#include "tobas_qt_tools/widgets/table_widget.hpp"

namespace qt
{
void TableWidget::removeAll()
{
  while (rowCount() > 0) {
    removeRow(0);
  }
}

void TableWidget::setColumnsWidth(int width)
{
  for (int col = 0; col < columnCount(); ++col) {
    setColumnWidth(col, width);
  }
}

void TableWidget::resizeHeightToContents()
{
  int total_height = 0;
  for (int row = 0; row < rowCount(); ++row) {
    total_height += rowHeight(row);
  }
  setFixedHeight(total_height);
}
}  // namespace qt
