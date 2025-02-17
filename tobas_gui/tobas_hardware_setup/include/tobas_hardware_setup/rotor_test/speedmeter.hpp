#pragma once

#include <QtQuickWidgets/QQuickWidget>

namespace gui
{
namespace hw
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
  double getTickmarkStepSize() const;
  double getValue() const;

  void setMaximumValue(double value);
  void setMinimumValue(double value);
  void setTickmarkStepSize(double value);
  void setValue(double value);

private:
  QObject* getGaugeObject() const;
};
}  // namespace hw
}  // namespace gui
