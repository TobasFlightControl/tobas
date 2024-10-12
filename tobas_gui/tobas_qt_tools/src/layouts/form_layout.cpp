#include <QWidget>
#include <QLabel>
#include <QHBoxLayout>

#include "tobas_qt_tools/layouts/form_layout.hpp"

namespace qt
{
void FormLayout::addVAlignedRow(QWidget* label, QWidget* field)
{
  const auto label_layout = new QHBoxLayout();
  label_layout->addWidget(label, 0, Qt::AlignLeft | Qt::AlignVCenter);

  const auto label_widget = new QWidget();
  label_widget->setLayout(label_layout);

  addRow(label_widget, field);
}

void FormLayout::addVAlignedRow(const QString& label_text, QWidget* field)
{
  addVAlignedRow(new QLabel(label_text), field);
}

void FormLayout::clear()
{
  while (rowCount() > 0)
    removeRow(0);
}

QWidget* FormLayout::getLabel(int row)
{
  const auto item = itemAt(row, QFormLayout::LabelRole);
  return item->widget();
}

QWidget* FormLayout::getWidget(int row)
{
  const auto item = itemAt(row, QFormLayout::FieldRole);
  return item->widget();
}
}  // namespace qt
