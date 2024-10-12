#pragma once

#include "./widget.hpp"

namespace qt
{
class ToggleSwitch : public qt::Widget
{
  Q_OBJECT

  using self = ToggleSwitch;
  using super = qt::Widget;

Q_SIGNALS:
  void toggled(bool checked);

public:
  explicit ToggleSwitch(QWidget* parent = nullptr);

  bool isChecked() const;
  void setChecked(bool checked);

  void setText(const QString& text);
  void ignoreMousePressEvent(bool ignore);

protected:
  void paintEvent(QPaintEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void resizeEvent(QResizeEvent* event) override;

private:
  bool checked_ = false;  // ON/OFFの状態
  QString text_ = "";
  bool ignore_mouse_press_event_ = false;

  void drawBackground(QPainter& painter);
  void drawSwitch(QPainter& painter);
  void drawText(QPainter& painter);
};
}  // namespace qt
