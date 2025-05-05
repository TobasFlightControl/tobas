#include "tobas_qt_tools/widgets/combo_box.hpp"

#include <QWheelEvent>
#include <QStandardItemModel>

#include "tobas_qt_tools/cast.hpp"

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
  if (index < 0) {
    throw std::runtime_error("\"" + text.toStdString() + "\" does not exist in the combo box choices.");
  }
  removeItem(index);
}

void ComboBox::setCurrentIndex(int index)
{
  if (index < 0 || count() <= index) {
    throw std::runtime_error("Index " + std::to_string(index) + " is out of range.");
  }
  super::setCurrentIndex(index);
}

void ComboBox::setCurrentText(const QString& text)
{
  const auto index = findText(text);
  if (index < 0) {
    throw std::runtime_error("\"" + text.toStdString() + "\" does not exist in the combo box choices.");
  }
  super::setCurrentIndex(index);
}

void ComboBox::sort()
{
  // 全てのアイテムをQStringListに集める
  QStringList items;
  for (int i = 0; i < count(); ++i) {
    items << itemText(i);
  }

  // 大文字小文字を区別せずに名前順に並べ替える
  items.sort(Qt::CaseInsensitive);

  // 一旦中身をクリアしてソート後のリストを追加し直す
  clear();
  addItems(items);
}

void ComboBox::setItemEnabled(int row, bool enabled)
{
  const auto model = qt::qConstPointerCast<QStandardItemModel>(this->model());
  const auto item = model->item(row);
  const auto cur_flags = item->flags();
  const auto new_flags = enabled ? cur_flags | Qt::ItemIsEnabled : cur_flags & ~Qt::ItemIsEnabled;
  item->setFlags(new_flags);
}

void ComboBox::setItemEnabled(const QString& text, bool enabled)
{
  setItemEnabled(findText(text), enabled);
}
}  // namespace qt
