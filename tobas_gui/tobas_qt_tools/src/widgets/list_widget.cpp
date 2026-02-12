#include "tobas_qt_tools/widgets/list_widget.hpp"

#include <QDebug>
#include <QDropEvent>
#include <QVariant>

namespace qt
{
bool ListWidget::contains(const QString& text) const
{
  const auto items = findItems(text, Qt::MatchExactly);
  return !items.empty();
}

void ListWidget::remove(QListWidgetItem* item)
{
  takeItem(row(item));
}

void ListWidget::setCurrentText(const QString& text)
{
  const auto items = findItems(text, Qt::MatchExactly);
  if (items.empty()) {
    qWarning() << text << "not found.";
    return;
  }

  const auto& item = items.first();
  setCurrentItem(item);
}

void ListWidget::deselect()
{
  const QSignalBlocker block(this);  // 負の行番号でシグナルが発行されるのを防ぐ
  setCurrentRow(-1);
}

void ListWidget::shrinkToContents()
{
  const auto rows = count();
  const auto row_height = sizeHintForRow(0);  // 行の高さ
  const auto frame = 2 * frameWidth();
  const auto margin = contentsMargins().top() + contentsMargins().bottom();

  setFixedHeight(rows * row_height + frame + margin);
  setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
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
