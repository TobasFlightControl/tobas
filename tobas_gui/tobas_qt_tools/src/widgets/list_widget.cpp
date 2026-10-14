// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/list_widget.hpp"

#include <QApplication>
#include <QDebug>
#include <QDropEvent>
#include <QPainter>
#include <QStyledItemDelegate>
#include <QVariant>

namespace tobas::qt
{
namespace
{
class RowNumberDelegate : public QStyledItemDelegate
{
public:
  explicit RowNumberDelegate(QObject* _parent) : QStyledItemDelegate(_parent)
  {
  }

  void paint(QPainter* _painter, const QStyleOptionViewItem& _option, const QModelIndex& _index) const override
  {
    QStyleOptionViewItem opt(_option);
    initStyleOption(&opt, _index);

    const auto text = opt.text;
    opt.text.clear();

    _painter->save();

    // Draw the background, selection state, focus, and so on normally.
    const auto widget = opt.widget;
    const auto style = widget ? widget->style() : QApplication::style();
    style->drawControl(QStyle::CE_ItemViewItem, &opt, _painter, widget);

    const QFontMetrics fm(opt.font);

    const auto nrows = _index.model() ? _index.model()->rowCount() : 0;
    const auto digits = QString::number(nrows).size();
    const QString max_no_text(digits, QLatin1Char('9'));
    const auto no_width = fm.horizontalAdvance(max_no_text) + 12;

    if (opt.state & QStyle::State_Selected) {
      _painter->setPen(opt.palette.color(QPalette::HighlightedText));
    }
    else {
      _painter->setPen(opt.palette.color(QPalette::Text));
    }

    const auto no_rect = opt.rect.adjusted(6, 0, -(opt.rect.width() - no_width), 0);
    const auto no_text = QString::number(_index.row() + 1);
    _painter->drawText(no_rect, Qt::AlignVCenter | Qt::AlignLeft, no_text);

    const auto text_rect = opt.rect.adjusted(no_width + 6, 0, -6, 0);
    const auto text_width = text_rect.width();
    _painter->drawText(text_rect, Qt::AlignVCenter | Qt::AlignLeft, fm.elidedText(text, Qt::ElideRight, text_width));

    _painter->restore();
  }
};
}  // namespace

bool ListWidget::contains(const QString& _text) const
{
  const auto items = findItems(_text, Qt::MatchExactly);
  return !items.empty();
}

void ListWidget::remove(QListWidgetItem* _item)
{
  takeItem(row(_item));
}

void ListWidget::setCurrentText(const QString& _text)
{
  const auto items = findItems(_text, Qt::MatchExactly);
  if (items.empty()) {
    qWarning() << _text << "not found.";
    return;
  }

  const auto& item = items.first();
  setCurrentItem(item);
}

void ListWidget::deselect()
{
  const QSignalBlocker block(this);  // Prevent signals from being emitted with a negative row number.
  setCurrentRow(-1);
}

void ListWidget::shrinkToContents()
{
  const auto rows = count();
  const auto row_height = sizeHintForRow(0);  // Row height.
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

void ListWidget::dropEvent(QDropEvent* _event)
{
  QListWidget::dropEvent(_event);
  Q_EMIT itemMoved(selectedItems().first());
}

bool ListWidgetItem::operator<(const QListWidgetItem& _rhs) const
{
  const auto ldata = data(Qt::UserRole);
  const auto rdata = _rhs.data(Qt::UserRole);
  return ldata.toString() < rdata.toString();
}
}  // namespace tobas::qt
