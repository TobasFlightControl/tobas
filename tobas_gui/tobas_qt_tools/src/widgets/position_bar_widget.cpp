#include <QPainter>
#include <QPaintEvent>

#include <tobas_math/core.hpp>

#include "tobas_qt_tools/widgets/position_bar_widget.hpp"
#include "tobas_qt_tools/font.hpp"

namespace qt
{
PositionBarWidget::PositionBarWidget(QWidget* parent) : super(parent)
{
}

PositionBarWidget::PositionBarWidget(double minimum, double maximum, QWidget* parent)
  : super(parent), minimum_(minimum), maximum_(maximum)
{
}

double PositionBarWidget::getMinimum() const
{
  return minimum_;
}

double PositionBarWidget::getMaximum() const
{
  return maximum_;
}

int PositionBarWidget::getLineWidth() const
{
  return line_width_;
}

int PositionBarWidget::getTextPSize() const
{
  return text_psize_;
}

const QString& PositionBarWidget::getText() const
{
  return text_.value();
}

double PositionBarWidget::getValue() const
{
  return value_.value();
}

double PositionBarWidget::getLower() const
{
  return lower_.value();
}

double PositionBarWidget::getUpper() const
{
  return upper_.value();
}

double PositionBarWidget::getMiddle() const
{
  return (lower_.value() + upper_.value()) / 2;
}

double PositionBarWidget::getRange() const
{
  return upper_.value() - lower_.value();
}

void PositionBarWidget::setMinimum(double minimum)
{
  minimum_ = minimum;
  update();
}

void PositionBarWidget::setMaximum(double maximum)
{
  maximum_ = maximum;
  update();
}

void PositionBarWidget::setFillRange(bool fill_range)
{
  fill_range_ = fill_range;
  update();
}

void PositionBarWidget::setLineWidth(int line_width)
{
  line_width_ = line_width;
  update();
}

void PositionBarWidget::setTextPSize(int text_psize)
{
  text_psize_ = text_psize;
  update();
}

void PositionBarWidget::setFillColor(Qt::GlobalColor color)
{
  fill_color_ = color;
  update();
}

void PositionBarWidget::setLimitLineColor(Qt::GlobalColor color)
{
  limit_line_color_ = color;
  update();
}

void PositionBarWidget::setValueLineColor(Qt::GlobalColor color)
{
  value_line_color_ = color;
  update();
}

void PositionBarWidget::setText(const QString& text)
{
  text_ = text;
  update();
}

void PositionBarWidget::setValue(double value)
{
  value_ = value;
  if (!lower_.has_value() || value < lower_.value())
    lower_ = value;
  if (!upper_.has_value() || value > upper_.value())
    upper_ = value;

  update();
}

void PositionBarWidget::setLower(double lower)
{
  lower_ = lower;
  update();
}

void PositionBarWidget::setUpper(double upper)
{
  upper_ = upper;
  update();
}

void PositionBarWidget::clear()
{
  text_.reset();
  value_.reset();
  lower_.reset();
  upper_.reset();
  update();
}

void PositionBarWidget::paintEvent(QPaintEvent* event)
{
  // QPainterはpaintEvent内でのみ定義できる
  QPainter painter(this);

  // 背景を描画
  painter.fillRect(event->rect(), Qt::white);

  // 枠を描画
  painter.save();
  painter.setPen(Qt::black);
  painter.drawRect(0, 0, width(), height());
  painter.restore();

  // 値の範囲を塗りつぶす
  if (fill_range_ && lower_.has_value() && upper_.has_value())
  {
    painter.save();
    drawRange(painter, lower_.value(), upper_.value());
    painter.restore();
  }

  // 値の位置を表示
  if (value_.has_value())
  {
    painter.save();
    drawValue(painter, value_.value());
    painter.restore();
  }

  // テキストを表示
  if (text_.has_value())
  {
    painter.save();
    drawText(painter, text_.value());
    painter.restore();
  }
}

void HPositionBarWidget::drawRange(QPainter& painter, double lower, double upper)
{
  // バーの位置を計算
  const int lower_pos = math::remap<double>(lower, getMinimum(), getMaximum(), 0, width());
  const int upper_pos = math::remap<double>(upper, getMinimum(), getMaximum(), 0, width());

  // 最小値と最大値の間を緑色で塗る
  painter.setBrush(fill_color_);
  painter.drawRect(lower_pos, 0, upper_pos - lower_pos, height());

  // 最小値と最大値の位置に黒色の線を描画
  painter.setPen(QPen(limit_line_color_, getLineWidth()));
  painter.drawLine(lower_pos, 0, lower_pos, height());
  painter.drawLine(upper_pos, 0, upper_pos, height());
}

void HPositionBarWidget::drawValue(QPainter& painter, double value)
{
  // バーの位置を計算
  const int value_pos = math::remap<double>(value, getMinimum(), getMaximum(), 0, width());

  // 現在値の位置に赤色の線を描画
  painter.setPen(QPen(value_line_color_, getLineWidth()));
  painter.drawLine(value_pos, 0, value_pos, height());
}

void HPositionBarWidget::drawText(QPainter& painter, const QString& text)
{
  painter.setPen(Qt::gray);
  painter.setFont(DefaultFont(getTextPSize()));
  painter.drawText(QRect(0, 0, width(), height()), Qt::AlignCenter, text);
}

void VPositionBarWidget::drawRange(QPainter& painter, double lower, double upper)
{
  // バーの位置を計算
  const int lower_pos = math::remap<double>(lower, getMinimum(), getMaximum(), 0, height());
  const int upper_pos = math::remap<double>(upper, getMinimum(), getMaximum(), 0, height());

  // 最小値と最大値の間を緑色で塗る
  painter.setBrush(fill_color_);
  painter.drawRect(0, lower_pos, width(), upper_pos - lower_pos);

  // 最小値と最大値の位置に黒色の線を描画
  painter.setPen(QPen(limit_line_color_, getLineWidth()));
  painter.drawLine(0, lower_pos, width(), lower_pos);
  painter.drawLine(0, upper_pos, width(), upper_pos);
}

void VPositionBarWidget::drawValue(QPainter& painter, double value)
{
  // バーの位置を計算
  const int value_pos = math::remap<double>(value, getMinimum(), getMaximum(), 0, height());

  // 現在値の位置に赤色の線を描画
  painter.setPen(QPen(value_line_color_, getLineWidth()));
  painter.drawLine(0, value_pos, width(), value_pos);
}

void VPositionBarWidget::drawText(QPainter& painter, const QString& text)
{
  // フォントを設定
  painter.setPen(Qt::gray);
  painter.setFont(DefaultFont(getTextPSize()));

  // ペインターの回転と移動を設定
  painter.translate(width() / 2, height() / 2);
  painter.rotate(90);

  // 回転した状態でテキストを描画
  QRect text_rect(-height() / 2, -width() / 2, height(), width());
  painter.drawText(text_rect, Qt::AlignCenter, text);
}
}  // namespace qt
