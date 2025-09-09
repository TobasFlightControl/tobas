#pragma once

#include "./widget.hpp"

namespace qt
{
class CircleWidget : public qt::Widget
{
  Q_OBJECT

  using self = CircleWidget;
  using super = qt::Widget;

public:
  explicit CircleWidget(QWidget* parent = nullptr);
  explicit CircleWidget(const QString& text, QWidget* parent = nullptr);

  void setFillColor(Qt::GlobalColor color);
  void setLineColor(Qt::GlobalColor color);
  void setLineWidth(int width);
  void setText(const QString& text);
  void setTextPointSize(int psize);

  int getDiameter() const;
  int getRadius() const;

  int calcMaxTextPointSize() const;

protected:
  void paintEvent(QPaintEvent* event) override;

private:
  Qt::GlobalColor fill_color_ = Qt::transparent;
  Qt::GlobalColor line_color_ = Qt::transparent;
  int line_width_ = 0;
  QString text_ = "";
  int text_psize_ = 0;

  void drawCircle(QPainter& painter);
  void drawText(QPainter& painter);
};
}  // namespace qt
