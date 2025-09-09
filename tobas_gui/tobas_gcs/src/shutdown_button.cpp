#include "tobas_gcs/shutdown_button.hpp"

#include <QGraphicsDropShadowEffect>

namespace gui
{
namespace gcs
{
ShutdownButton::ShutdownButton(int radius) : super("⏻")
{
  setCheckable(true);

  const auto font_size = radius * 3 / 2;
  const auto diameter = radius * 2;

  // ボタンの外観をスタイルシートで設定
  QString qss = R"(
    QPushButton {
      background-color: qlineargradient(spread:pad, x1:0.5, y1:0, x2:0.5, y2:1, stop:0 #FF6666, stop:1 #990000);
      border: 2px solid #440000;
      border-radius: %1px;
      color: white;
      font-size: %2px;
      width: %3px;
      height: %3px;
    }
    QPushButton:hover {
      background-color: qlineargradient(spread:pad, x1:0.5, y1:0, x2:0.5, y2:1, stop:0 #FF9999, stop:1 #CC0000);
    }
    QPushButton:pressed, QPushButton:checked {
      background-color: qlineargradient(spread:pad, x1:0.5, y1:0, x2:0.5, y2:1, stop:0 #660000, stop:1 #330000);
      padding-top: 2px;
    }
    QPushButton:disabled {
      background-color: qlineargradient(
        spread:pad,
        x1:0.5,
        y1:0,
        x2:0.5,
        y2:1,
        stop:0 rgba(255,102,102,0.5),
        stop:1 rgba(153,0,0,0.5));
      border-color: rgba(0,0,0,0.5);
      color: rgba(255,255,255,0.5);
    }
  )";
  qss = qss.arg(radius).arg(font_size).arg(diameter);
  setStyleSheet(qss);
}
}  // namespace gcs
}  // namespace gui
