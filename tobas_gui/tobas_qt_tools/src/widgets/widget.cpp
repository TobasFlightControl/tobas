#include <QPainter>

#include "tobas_qt_tools/widgets/widget.hpp"

namespace qt
{
QPoint Widget::getCenter() const
{
  return QPoint(width() / 2, height() / 2);
}

int Widget::calcMaxTextPointSize(const QString& text, const QPoint& center) const
{
  int point_size = 1;
  auto font = this->font();

  while (true) {
    font.setPointSize(point_size++);
    QFontMetrics fm(font);

    const auto text_width = fm.horizontalAdvance(text);
    const auto text_height = fm.height();

    static constexpr double kMargin = 0.1;
    if (center.x() - text_width / 2 < width() * kMargin || width() * (1 - kMargin) < center.x() + text_width / 2) {
      break;
    }
    if (center.y() - text_height / 2 < height() * kMargin || height() * (1 - kMargin) < center.y() + text_height / 2) {
      break;
    }
  }

  return point_size;
}

void Widget::drawText(QPainter& painter, const QString& text, const QPoint& center, const QFont& font)
{
  painter.save();

  painter.setFont(font);
  QFontMetrics fm(font);

  const auto text_width = fm.horizontalAdvance(text);
  const auto text_height = fm.height();
  const auto x = center.x() - text_width / 2;
  const auto y = center.y() + text_height / 2 - fm.descent();  // ベースライン補正
  painter.drawText(x, y, text);

  painter.restore();
}

void Widget::drawText(QPainter& painter, const QString& text, const QPoint& center, int point_size)
{
  auto font = this->font();
  font.setPointSize(point_size);
  drawText(painter, text, center, font);
}

void Widget::drawMaximumText(QPainter& painter, const QString& text, const QPoint& center)
{
  if (text.isEmpty()) {
    return;
  }

  const auto point_size = calcMaxTextPointSize(text, center);
  drawText(painter, text, center, point_size);
}

void Widget::drawMaximumText(QPainter& painter, const QString& text)
{
  drawMaximumText(painter, text, getCenter());
}
}  // namespace qt
