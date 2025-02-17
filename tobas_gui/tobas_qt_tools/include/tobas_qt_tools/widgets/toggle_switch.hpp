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
  void setOnColor(const QColor& color);
  void setOffColor(const QColor& color);

  void ignoreMousePressEvent(bool ignore);

protected:
  void paintEvent(QPaintEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void resizeEvent(QResizeEvent* event) override;

private:
  bool checked_ = false;  // ON/OFFの状態
  bool ignore_mouse_press_event_ = false;
  QString text_ = "";
  QColor on_color_ = Qt::green;
  QColor off_color_ = Qt::gray;

  void drawBackground(QPainter& painter);
  void drawSwitch(QPainter& painter);
  void drawText(QPainter& painter);
};
}  // namespace qt
