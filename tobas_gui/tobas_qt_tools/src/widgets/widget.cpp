#include <QPainter>

#include "tobas_qt_tools/widgets/widget.hpp"

namespace qt
{
QPoint Widget::getCenter() const
{
  return QPoint(width() / 2, height() / 2);
}

void Widget::drawMaximumText(QPainter& painter, const QString& text)
{
  drawMaximumText(painter, text, getCenter());
}

void Widget::drawMaximumText(QPainter& painter, const QString& text, const QPoint& center)
{
  static constexpr double kMargin = 0.1;

  if (text.isEmpty())
    return;

  painter.save();

  // フォントサイズを調整
  int font_size = 1;
  auto font = painter.font();
  QFontMetrics fm(font);
  while (true)
  {
    const auto text_width = fm.horizontalAdvance(text);
    const auto text_height = fm.height();

    if (center.x() - text_width / 2 < width() * kMargin || width() * (1 - kMargin) < center.x() + text_width / 2)
      break;
    if (center.y() - text_height / 2 < height() * kMargin || height() * (1 - kMargin) < center.y() + text_height / 2)
      break;

    font.setPointSize(++font_size);
    painter.setFont(font);
    fm = painter.fontMetrics();  // 更新されたフォントで再計算
  }

  // テキストを描画
  painter.setPen(Qt::black);
  const auto text_width = fm.horizontalAdvance(text);
  const auto text_height = fm.height();
  const auto x = center.x() - text_width / 2;
  const auto y = center.y() + text_height / 2 - fm.descent();  // ベースライン補正
  painter.drawText(x, y, text);

  painter.restore();
}
}  // namespace qt
