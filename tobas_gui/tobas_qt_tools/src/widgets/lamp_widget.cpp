#include <format>

#include "tobas_qt_tools/widgets/lamp_widget.hpp"

using namespace std;

namespace qt
{
LampWidget::LampWidget(QWidget* parent) : super(parent)
{
}

void LampWidget::setColor(const RGBColor& color)
{
  // 色が変化しないなら何もしない
  if (color == color_)
    return;

  // 描画
  const auto radius = sizeHint().height() / 2;
  const auto grad = color.mean(RGBColor::White());
  const auto qss = format(QSS, radius, grad.r, grad.g, grad.b, color.r, color.g, color.b, color.r, color.g, color.b);
  setStyleSheet(QString::fromStdString(qss));

  // 新しい色に更新
  color_ = color;
}
}  // namespace qt
