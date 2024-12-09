#pragma once

#include <QtQuickWidgets/QQuickWidget>

namespace gui
{
namespace control_system
{
class SpeedmeterWidget : public QQuickWidget
{
  Q_OBJECT

  using self = SpeedmeterWidget;
  using super = QQuickWidget;

public:
  explicit SpeedmeterWidget();

  QString getBackgroundColor() const;
  double getMaximumValue() const;
  double getMinimumValue() const;
  double getStepSize() const;
  double getValue() const;
  QString getTopText() const;
  QString getBottomText() const;

  void setBackgroundColor(const QString& text);
  void setMaximumValue(double value);
  void setMinimumValue(double value);
  void setStepSize(double value);
  void setValue(double value);
  void setTopText(const QString& text);
  void setBottomText(const QString& text);

private:
  QObject* getRectangleObject() const;
  QObject* getGaugeObject() const;
  QObject* getTopTextObject() const;
  QObject* getBottomTextObject() const;
};
}  // namespace control_system
}  // namespace gui
