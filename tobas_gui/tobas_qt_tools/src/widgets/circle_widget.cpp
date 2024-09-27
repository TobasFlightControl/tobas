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
    drawMaximumText(painter, text_);
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
}  // namespace qt
