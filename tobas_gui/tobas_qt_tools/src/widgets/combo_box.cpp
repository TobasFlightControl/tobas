#include "tobas_qt_tools/widgets/combo_box.hpp"

namespace qt
{
void ComboBox::wheelEvent(QWheelEvent* event)
{
  event->ignore();
}

bool ComboBox::contains(const QString& text) const
{
  return findText(text) >= 0;
}

void ComboBox::removeText(const QString& text)
{
  const auto index = findText(text);
  if (index < 0)
    throw std::runtime_error("\"" + text.toStdString() + "\" does not exist in the combo box choices.");
  removeItem(index);
}

void ComboBox::setCurrentIndex(int index)
{
  if (index < 0 || count() <= index)
    throw std::runtime_error("Index " + std::to_string(index) + " is out of range.");
  super::setCurrentIndex(index);
}

void ComboBox::setCurrentText(const QString& text)
{
  const auto index = findText(text);
  if (index < 0)
    throw std::runtime_error("\"" + text.toStdString() + "\" does not exist in the combo box choices.");
  super::setCurrentIndex(index);
}
}  // namespace qt
