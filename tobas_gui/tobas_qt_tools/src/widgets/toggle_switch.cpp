#include <QPainter>
#include <QResizeEvent>
#include <QDebug>

#include "tobas_qt_tools/widgets/toggle_switch.hpp"

namespace qt
{
ToggleSwitch::ToggleSwitch(QWidget* parent) : super(parent)
{
}

bool ToggleSwitch::isChecked() const
{
  return checked_;
}

void ToggleSwitch::setChecked(bool checked)
{
  checked_ = checked;
}

void ToggleSwitch::setText(const QString& text)
{
  text_ = text;
}

void ToggleSwitch::ignoreMousePressEvent(bool ignore)
{
  ignore_mouse_press_event_ = ignore;
}

void ToggleSwitch::paintEvent(QPaintEvent*)
{
  if (width() < height())
  {
    qWarning() << "The height of toggle switch is greater than the width.";
    return;
  }

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  painter.save();
  drawBackground(painter);
  painter.restore();

  painter.save();
  drawSwitch(painter);
  painter.restore();

  painter.save();
  drawText(painter);
  painter.restore();
}

void ToggleSwitch::mousePressEvent(QMouseEvent*)
{
  if (ignore_mouse_press_event_)
    return;

  checked_ = !checked_;    // 状態をトグル
  emit toggled(checked_);  // トグル状態が変わったことを通知
}

void ToggleSwitch::resizeEvent(QResizeEvent* event)
{
  const auto w = event->size().width();
  const auto h = event->size().height();

  // アスペクト比を2:1よりも横長に保つ
  // 足りない方の長さで調整するとウィジェットが消滅してしまうため，必ず足りない方の長さを据え置きでもう片方で合わせる．
  if (w < 2 * h)
    resize(w, w / 2);

  super::resizeEvent(event);
}

void ToggleSwitch::drawBackground(QPainter& painter)
{
  if (checked_)
    painter.setBrush(Qt::green);  // ONの時の色
  else
    painter.setBrush(Qt::gray);  // OFFの時の色

  painter.setPen(Qt::NoPen);
  painter.drawRoundedRect(0, 0, width(), height(), height() / 2, height() / 2);
}

void ToggleSwitch::drawSwitch(QPainter& painter)
{
  painter.setBrush(Qt::white);
  const auto x = checked_ ? 0 : width() - height();
  painter.drawEllipse(x, 0, height(), height());  // スイッチ部分の描画
}

void ToggleSwitch::drawText(QPainter& painter)
{
  const auto cx = checked_ ? (width() + height()) / 2 : (width() - height()) / 2;
  const auto cy = height() / 2;
  drawMaximumText(painter, text_, QPoint(cx, cy));
}
}  // namespace qt
