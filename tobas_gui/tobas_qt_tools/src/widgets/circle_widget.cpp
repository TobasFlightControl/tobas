#include <QPainter>

#include "tobas_qt_tools/widgets/circle_widget.hpp"

namespace qt
{
CircleWidget::CircleWidget(QWidget* parent) : super(parent)
{
}

CircleWidget::CircleWidget(const QString& text, QWidget* parent) : super(parent), text_(text)
{
}

void CircleWidget::setColor(Qt::GlobalColor color)
{
  color_ = color;
}

void CircleWidget::setText(const QString& text)
{
  text_ = text;
}

int CircleWidget::getDiameter() const
{
  return std::min(width(), height());
}

int CircleWidget::getRadius() const
{
  return getDiameter() / 2;
}

void CircleWidget::paintEvent(QPaintEvent*)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  painter.save();
  drawCircle(painter);
  painter.restore();

  if (!text_.isEmpty())
  {
    painter.save();
    drawText(painter);
    painter.restore();
  }
}

void CircleWidget::drawCircle(QPainter& painter)
{
  QRadialGradient gradient(getCenter(), getRadius());
  gradient.setColorAt(0, QColor(color_).lighter());  // 中心
  gradient.setColorAt(1, color_);                    // 外側

  painter.setPen(Qt::NoPen);
  painter.setBrush(QBrush(gradient));

  const auto r = getRadius();
  painter.drawEllipse(getCenter(), r, r);
}

void CircleWidget::drawText(QPainter& painter)
{
  // テキストを描画するためのフォントサイズを調整
  adjustFontSize(painter);

  // 黒いテキストを中央に描画
  painter.setPen(Qt::black);
  QFontMetrics fm = painter.fontMetrics();
  const auto text_width = fm.horizontalAdvance(text_);
  const auto text_height = fm.height();
  const auto x = (width() - text_width) / 2;
  const auto y = (height() + text_height) / 2 - fm.descent();  // ベースライン補正
  painter.drawText(x, y, text_);
}

void CircleWidget::adjustFontSize(QPainter& painter)
{
  const auto diameter = getDiameter();
  int font_size = 1;
  QFont font = painter.font();

  // テキストが円の中に収まる最大のフォントサイズを計算
  QFontMetrics fm(font);
  while (fm.horizontalAdvance(text_) < diameter * 0.8 && fm.height() < diameter * 0.8)
  {
    font.setPointSize(++font_size);
    painter.setFont(font);
    fm = painter.fontMetrics();  // 更新されたフォントで再計算
  }

  // 最大サイズを超えた場合はフォントサイズを1つ戻す
  font.setPointSize(font_size - 1);
  painter.setFont(font);
}
}  // namespace qt
