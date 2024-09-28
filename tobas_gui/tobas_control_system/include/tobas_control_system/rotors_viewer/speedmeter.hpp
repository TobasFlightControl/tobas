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

  double getMaximumValue() const;
  double getMinimumValue() const;
  double getStepSize() const;
  double getValue() const;

  void setMaximumValue(double max_value);
  void setMinimumValue(double min_value);
  void setStepSize(double step_size);
  void setValue(double value);

private:
  QObject* getGauge() const;
};
}  // namespace control_system
}  // namespace gui
