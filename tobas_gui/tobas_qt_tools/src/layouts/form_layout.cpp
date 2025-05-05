#include "tobas_qt_tools/layouts/form_layout.hpp"

#include <QWidget>
#include <QLabel>
#include <QHBoxLayout>

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

void FormLayout::addStretch()
{
  addItem(new QSpacerItem(0, 0, QSizePolicy::Minimum, QSizePolicy::Expanding));
}

void FormLayout::clear()
{
  while (rowCount() > 0) {
    removeRow(0);
  }
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

void FormLayout::setEnabled(int row, bool enabled)
{
  getLabel(row)->setEnabled(enabled);
  getWidget(row)->setEnabled(enabled);
}

void FormLayout::setVisible(int row, bool visible)
{
  getLabel(row)->setVisible(visible);
  getWidget(row)->setVisible(visible);
}
}  // namespace qt
