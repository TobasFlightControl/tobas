#include "tobas_qt_tools/widgets/list_widget.hpp"

#include <QDropEvent>
#include <QVariant>

namespace qt
{
void ListWidget::remove(QListWidgetItem* item)
{
  takeItem(row(item));
}

bool ListWidget::contains(const QString& text)
{
  const auto items = findItems(text, Qt::MatchExactly);
  return items.size() > 0;
}

QListWidgetItem* ListWidget::selectedItem()
{
  const auto& selected_items = selectedItems();
  if (selected_items.size() > 0) {
    return selected_items.first();
  }
  else {
    return nullptr;
  }
}

void ListWidget::dropEvent(QDropEvent* event)
{
  QListWidget::dropEvent(event);
  Q_EMIT itemMoved(selectedItems().first());
}

bool ListWidgetItem::operator<(const QListWidgetItem& rhs) const
{
  const auto ldata = this->data(Qt::UserRole);
  const auto rdata = rhs.data(Qt::UserRole);
  return ldata.toString() < rdata.toString();
}
}  // namespace qt
