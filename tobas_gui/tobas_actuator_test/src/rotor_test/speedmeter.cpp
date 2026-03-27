#include "tobas_actuator_test/rotor_test/speedmeter.hpp"

#include <QtQml/QQmlContext>
#include <QtQuick/QQuickItem>

#include <tobas_std_tools/check.hpp>

#include "tobas_actuator_test/util.hpp"

namespace tobas
{
namespace gui
{
namespace at
{
SpeedmeterWidget::SpeedmeterWidget()
{
  // サイズポリシーとリサイズモードの指定 (しないとウィジェットが潰れてしまう)
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
  setResizeMode(QQuickWidget::SizeRootObjectToView);

  // QMLを読み込む
  const auto qml_path = getPkgShareDir() / "qml/SpeedMeter.qml";
  setSource(QUrl::fromLocalFile(QString::fromStdString(qml_path)));
}

double SpeedmeterWidget::getMaximumValue() const
{
  return getGaugeObject()->property("maximumValue").value<double>();
}

double SpeedmeterWidget::getMinimumValue() const
{
  return getGaugeObject()->property("minimumValue").value<double>();
}

double SpeedmeterWidget::getTickmarkStepSize() const
{
  return getGaugeObject()->property("tickmarkStepSize").value<double>();
}

double SpeedmeterWidget::getValue() const
{
  return getGaugeObject()->property("value").value<double>();
}

void SpeedmeterWidget::setMaximumValue(double value)
{
  QMetaObject::invokeMethod(rootObject(), "setMaximumValue", Q_ARG(double, value));
}

void SpeedmeterWidget::setMinimumValue(double value)
{
  QMetaObject::invokeMethod(rootObject(), "setMinimumValue", Q_ARG(double, value));
}

void SpeedmeterWidget::setTickmarkStepSize(double value)
{
  QMetaObject::invokeMethod(rootObject(), "setTickmarkStepSize", Q_ARG(double, value));
}

void SpeedmeterWidget::setValue(double value)
{
  QMetaObject::invokeMethod(rootObject(), "setValue", Q_ARG(double, value));
}

QObject* SpeedmeterWidget::getGaugeObject() const
{
  const auto gauge = rootObject()->findChild<QObject*>("gauge");
  TOBAS_CHECK(gauge);
  return gauge;
}
}  // namespace at
}  // namespace gui
}  // namespace tobas
