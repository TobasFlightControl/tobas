#include "tobas_qt_tools/widgets/circle_widget.hpp"

#include <QDebug>
#include <QPainter>

namespace tobas
{
namespace qt
{
CircleWidget::CircleWidget(QWidget* parent) : super(parent)
{
}

CircleWidget::CircleWidget(const QString& text, QWidget* parent) : super(parent), text_(text)
{
}

void CircleWidget::setFillColor(Qt::GlobalColor color)
{
  fill_color_ = color;
  update();
}

void CircleWidget::setLineColor(Qt::GlobalColor color)
{
  line_color_ = color;
  update();
}

void CircleWidget::setLineWidth(int width)
{
  if (width < 0) {
    qWarning() << "Line width must be non-negative.";
    return;
  }

  line_width_ = width;
  update();
}

void CircleWidget::setText(const QString& text)
{
  text_ = text;
  update();
}

void CircleWidget::setTextPointSize(int psize)
{
  if (psize < 0) {
    qWarning() << "Text point size must be non-negative.";
    return;
  }

  text_psize_ = psize;
  update();
}

int CircleWidget::getDiameter() const
{
  return std::min(width(), height());
}

int CircleWidget::getRadius() const
{
  return getDiameter() / 2;
}

int CircleWidget::calcMaxTextPointSize() const
{
  const auto diameter = getDiameter();

  int psize = 1;
  auto font = this->font();

  while (true) {
    font.setPointSize(psize++);
    QFontMetrics fm(font);

    const auto text_width = fm.horizontalAdvance(text_);
    if (text_width > 0.9 * diameter) {
      break;
    }
  }

  return psize;
}

void CircleWidget::paintEvent(QPaintEvent*)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  drawCircle(painter);

  if (!text_.isEmpty()) {
    drawText(painter);
  }
}

void CircleWidget::drawCircle(QPainter& painter)
{
  painter.save();

  const auto center = getCenter();
  const auto radius = getRadius() - line_width_ / 2;

  // 塗りつぶし用のブラシを設定
  if (fill_color_ != Qt::transparent) {
    QRadialGradient gradient(center, radius);
    gradient.setColorAt(0, QColor(fill_color_).lighter());  // 中心
    gradient.setColorAt(1, fill_color_);                    // 外側
    painter.setBrush(QBrush(gradient));
  }
  else {
    painter.setBrush(Qt::NoBrush);
  }

  // 輪郭用のペンを設定
  if (line_color_ != Qt::transparent && line_width_ > 0) {
    painter.setPen(QPen(line_color_, line_width_, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  }
  else {
    painter.setPen(Qt::NoPen);
  }

  // 中心と半径を指定して正円を描く
  painter.drawEllipse(center, radius, radius);

  painter.restore();
}

void CircleWidget::drawText(QPainter& painter)
{
  painter.save();

  const auto center = getCenter();

  if (text_psize_ > 0) {
    super::drawText(painter, text_, center, text_psize_);
  }
  else {
    drawMaximumText(painter, text_, center);
  }

  painter.restore();
}
}  // namespace qt
}  // namespace tobas
