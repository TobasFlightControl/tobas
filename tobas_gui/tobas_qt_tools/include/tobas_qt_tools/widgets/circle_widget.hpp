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

  void setColor(Qt::GlobalColor color);
  void setText(const QString& text);

  int getDiameter() const;
  int getRadius() const;

protected:
  void paintEvent(QPaintEvent* event) override;

private:
  Qt::GlobalColor color_ = Qt::black;
  QString text_ = "";

  void drawCircle(QPainter& painter);
  void drawText(QPainter& painter);

  /* 円のサイズに合わせてフォントサイズを調整する． */
  void adjustFontSize(QPainter& painter);
};
}  // namespace qt
