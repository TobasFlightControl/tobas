#pragma once

#include <QtQuickWidgets/QQuickWidget>

namespace gui
{
namespace hardware_setup
{
class SpeedmeterWidget : public QQuickWidget
{
  Q_OBJECT

  using self = SpeedmeterWidget;
  using super = QQuickWidget;

public:
  explicit SpeedmeterWidget();

  double getMaximumValue() const;
  double getMinimumValue() const;
  double getValue() const;

  void setMaximumValue(double value);
  void setMinimumValue(double value);
  void setValue(double value);

private:
  QObject* getGaugeObject() const;
};
}  // namespace hardware_setup
}  // namespace gui
