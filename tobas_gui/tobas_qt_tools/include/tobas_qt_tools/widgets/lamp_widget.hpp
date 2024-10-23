#pragma once

#include <QLabel>

#include "tobas_qt_tools/rgb_color.hpp"

namespace qt
{
class LampWidget : public QLabel
{
  Q_OBJECT

  using super = QLabel;

  static constexpr char QSS[] = "QLabel {{"
                                "border: 2px solid lightgray;"
                                "border-radius: {}px;"
                                "background-color:"
                                "QLinearGradient("
                                "y1: 0, y2: 1,"
                                "stop: 0 WHITE,"
                                "stop: 0.2 #{:02X}{:02X}{:02X},"
                                "stop: 0.8 #{:02X}{:02X}{:02X},"
                                "stop: 1 #{:02X}{:02X}{:02X}"
                                ");"
                                "}}";

public:
  explicit LampWidget(QWidget* parent = nullptr);
  explicit LampWidget(const QString& text, QWidget* parent = nullptr);

  void setColor(const RGBColor& color);

private:
  RGBColor c_ = RGBColor::Black();

  void draw();
};
}  // namespace qt
