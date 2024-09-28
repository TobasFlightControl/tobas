#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <QtQuick/QQuickItem>
#include <QtQml/QQmlContext>

#include "tobas_control_system/rotors_viewer/speedmeter.hpp"
#include "tobas_control_system/constants.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace control_system
{
SpeedmeterWidget::SpeedmeterWidget()
{
  // サイズポリシーとリサイズモードの指定 (しないとウィジェットが潰れてしまう)
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
  setResizeMode(QQuickWidget::SizeRootObjectToView);

  // QMLを読み込む
  const fs::path pkg_path(ament_index_cpp::get_package_share_directory(kPkgName));
  const auto qml_path = pkg_path / "qml/SpeedMeter.qml";
  setSource(QUrl::fromLocalFile(QString::fromStdString(qml_path)));
}

double SpeedmeterWidget::getMaximumValue() const
{
  return getGauge()->property("maximumValue").value<double>();
}

double SpeedmeterWidget::getMinimumValue() const
{
  return getGauge()->property("minimumValue").value<double>();
}

double SpeedmeterWidget::getStepSize() const
{
  return getGauge()->property("stepSize").value<double>();
}

double SpeedmeterWidget::getValue() const
{
  return getGauge()->property("value").value<double>();
}

void SpeedmeterWidget::setMaximumValue(double max_value)
{
  QMetaObject::invokeMethod(rootObject(), "setMaximumValue", Q_ARG(double, max_value));
}

void SpeedmeterWidget::setMinimumValue(double min_value)
{
  QMetaObject::invokeMethod(rootObject(), "setMinimumValue", Q_ARG(double, min_value));
}

void SpeedmeterWidget::setStepSize(double step_size)
{
  QMetaObject::invokeMethod(rootObject(), "setStepSize", Q_ARG(double, step_size));
}

void SpeedmeterWidget::setValue(double value)
{
  QMetaObject::invokeMethod(rootObject(), "setValue", Q_ARG(double, value));
}

QObject* SpeedmeterWidget::getGauge() const
{
  return rootObject()->findChild<QObject*>("gauge");
}
}  // namespace control_system
}  // namespace gui
