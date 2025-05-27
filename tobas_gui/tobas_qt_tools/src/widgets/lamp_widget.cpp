#include "tobas_qt_tools/widgets/lamp_widget.hpp"

#include <format>

using namespace std;

namespace qt
{
LampWidget::LampWidget(QWidget* parent) : super(parent)
{
  draw();
}

LampWidget::LampWidget(const QString& text, QWidget* parent) : super(text, parent)
{
  draw();
}

void LampWidget::setColor(const RGBColor& color)
{
  // 色が変化しないなら何もしない
  if (color == c_) {
    return;
  }

  // 新しい色に更新
  c_ = color;

  // 描画
  draw();
}

void LampWidget::draw()
{
  const auto radius = sizeHint().height() / 2;
  const auto grad = c_.mean(RGBColor::White());
  const auto qss = format(QSS, radius, grad.r, grad.g, grad.b, c_.r, c_.g, c_.b, c_.r, c_.g, c_.b);
  setStyleSheet(QString::fromStdString(qss));
}
}  // namespace qt
