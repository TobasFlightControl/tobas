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

QString SpeedmeterWidget::getBackgroundColor() const
{
  return getRectangleObject()->property("color").value<QString>();
}

double SpeedmeterWidget::getMaximumValue() const
{
  return getGaugeObject()->property("maximumValue").value<double>();
}

double SpeedmeterWidget::getMinimumValue() const
{
  return getGaugeObject()->property("minimumValue").value<double>();
}

double SpeedmeterWidget::getStepSize() const
{
  return getGaugeObject()->property("stepSize").value<double>();
}

double SpeedmeterWidget::getValue() const
{
  return getGaugeObject()->property("value").value<double>();
}

QString SpeedmeterWidget::getTopText() const
{
  return getTopTextObject()->property("text").value<QString>();
}

QString SpeedmeterWidget::getBottomText() const
{
  return getBottomTextObject()->property("text").value<QString>();
}

void SpeedmeterWidget::setBackgroundColor(const QString& text)
{
  QMetaObject::invokeMethod(rootObject(), "setBackgroundColor", Q_ARG(QString, text));
}

void SpeedmeterWidget::setMaximumValue(double value)
{
  QMetaObject::invokeMethod(rootObject(), "setMaximumValue", Q_ARG(double, value));
}

void SpeedmeterWidget::setMinimumValue(double value)
{
  QMetaObject::invokeMethod(rootObject(), "setMinimumValue", Q_ARG(double, value));
}

void SpeedmeterWidget::setStepSize(double value)
{
  QMetaObject::invokeMethod(rootObject(), "setStepSize", Q_ARG(double, value));
}

void SpeedmeterWidget::setValue(double value)
{
  QMetaObject::invokeMethod(rootObject(), "setValue", Q_ARG(double, value));
}

void SpeedmeterWidget::setTopText(const QString& text)
{
  QMetaObject::invokeMethod(rootObject(), "setTopText", Q_ARG(QString, text));
}

void SpeedmeterWidget::setBottomText(const QString& text)
{
  QMetaObject::invokeMethod(rootObject(), "setBottomText", Q_ARG(QString, text));
}

QObject* SpeedmeterWidget::getRectangleObject() const
{
  return rootObject()->findChild<QObject*>("rectangle");
}

QObject* SpeedmeterWidget::getGaugeObject() const
{
  return rootObject()->findChild<QObject*>("gauge");
}

QObject* SpeedmeterWidget::getTopTextObject() const
{
  return rootObject()->findChild<QObject*>("top_text");
}

QObject* SpeedmeterWidget::getBottomTextObject() const
{
  return rootObject()->findChild<QObject*>("bottom_text");
}
}  // namespace control_system
}  // namespace gui
