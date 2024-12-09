#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <QtQuick/QQuickItem>
#include <QtQml/QQmlContext>

#include "tobas_hardware_setup/rotor_test/speedmeter.hpp"
#include "tobas_hardware_setup/constants.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace hardware_setup
{
SpeedmeterWidget::SpeedmeterWidget()
{
  // サイズポリシーとリサイズモードの指定 (しないとウィジェットが潰れてしまう)
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
  setResizeMode(QQuickWidget::SizeRootObjectToView);

  // QMLを読み込む
  const fs::path pkg_path(ament_index_cpp::get_package_share_directory(kPackageName));
  const auto qml_path = pkg_path / "qml/SpeedMeter.qml";
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

void SpeedmeterWidget::setValue(double value)
{
  QMetaObject::invokeMethod(rootObject(), "setValue", Q_ARG(double, value));
}

QObject* SpeedmeterWidget::getGaugeObject() const
{
  return rootObject()->findChild<QObject*>("gauge");
}
}  // namespace hardware_setup
}  // namespace gui
