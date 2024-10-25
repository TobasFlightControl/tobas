#include <QGraphicsDropShadowEffect>

#include "tobas_gui_core/power_button.hpp"

namespace gui
{
namespace core
{
PowerButton::PowerButton(int radius) : super("⏻")
{
  const auto font_size = radius * 3 / 2;
  const auto diameter = radius * 2;

  // ボタンの外観をスタイルシートで設定
  QString qss = R"(
    QPushButton {
      background-color: qlineargradient(spread:pad, x1:0.5, y1:0, x2:0.5, y2:1, stop:0 #FF4D4D, stop:1 #990000);
      border: none;
      border-radius: %1px;
      color: white;
      font-size: %2px;
      width: %3px;
      height: %3px;
    }
    QPushButton:hover {
      background-color: qlineargradient(spread:pad, x1:0.5, y1:0, x2:0.5, y2:1, stop:0 #FF3333, stop:1 #990000);
    }
    QPushButton:pressed {
      background-color: qlineargradient(spread:pad, x1:0.5, y1:0, x2:0.5, y2:1, stop:0 #CC0000, stop:1 #660000);
      padding-top: 2px;
    }
  )";
  qss = qss.arg(radius).arg(font_size).arg(diameter);
  setStyleSheet(qss);
}
}  // namespace core
}  // namespace gui
