// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/list_widget.hpp"

#include <QApplication>
#include <QDebug>
#include <QDropEvent>
#include <QPainter>
#include <QStyledItemDelegate>
#include <QVariant>

namespace tobas
{
namespace qt
{
namespace
{
class RowNumberDelegate : public QStyledItemDelegate
{
public:
  explicit RowNumberDelegate(QObject* parent) : QStyledItemDelegate(parent)
  {
  }

  void paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const override
  {
    QStyleOptionViewItem opt(option);
    initStyleOption(&opt, index);

    const auto text = opt.text;
    opt.text.clear();

    painter->save();

    // 背景，選択状態，フォーカスなどを通常通り描画
    const auto widget = opt.widget;
    const auto style = widget ? widget->style() : QApplication::style();
    style->drawControl(QStyle::CE_ItemViewItem, &opt, painter, widget);

    const QFontMetrics fm(opt.font);

    const auto nrows = index.model() ? index.model()->rowCount() : 0;
    const auto digits = QString::number(nrows).size();
    const QString max_no_text(digits, QLatin1Char('9'));
    const auto no_width = fm.horizontalAdvance(max_no_text) + 12;

    if (opt.state & QStyle::State_Selected) {
      painter->setPen(opt.palette.color(QPalette::HighlightedText));
    }
    else {
      painter->setPen(opt.palette.color(QPalette::Text));
    }

    const auto no_rect = opt.rect.adjusted(6, 0, -(opt.rect.width() - no_width), 0);
    const auto no_text = QString::number(index.row() + 1);
    painter->drawText(no_rect, Qt::AlignVCenter | Qt::AlignLeft, no_text);

    const auto text_rect = opt.rect.adjusted(no_width + 6, 0, -6, 0);
    const auto text_width = text_rect.width();
    painter->drawText(text_rect, Qt::AlignVCenter | Qt::AlignLeft, fm.elidedText(text, Qt::ElideRight, text_width));

    painter->restore();
  }
};
}  // namespace

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

void ListWidget::showRowNumber()
{
  setItemDelegate(new RowNumberDelegate(this));
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
}  // namespace tobas
