#include "tobas_qt_tools/layouts/form_layout.hpp"

namespace qt
{
void FormLayout::clear()
{
  while (rowCount() > 0)
    removeRow(0);
}

QWidget* FormLayout::getLabel(int row)
{
  auto item = itemAt(row, QFormLayout::LabelRole);
  return item->widget();
}

QWidget* FormLayout::getWidget(int row)
{
  auto item = itemAt(row, QFormLayout::FieldRole);
  return item->widget();
}
}  // namespace qt
