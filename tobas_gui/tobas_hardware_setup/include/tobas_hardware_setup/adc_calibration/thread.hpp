#pragma once

#include <rclcpp/node.hpp>
#include <QThread>

#include <tobas_algorithm/kahan.hpp>
#include <tobas_msgs/msg/adc.hpp>

namespace gui
{
namespace hardware_setup
{
class ADCCalibrationThread : public QThread
{
  Q_OBJECT

  using self = ADCCalibrationThread;
  using super = QThread;

  static constexpr size_t kDataCount = 200;
  static constexpr double kCollectDataTimeout = 10.;  // [s]

Q_SIGNALS:
  void finished(bool success, const QString& message);

public:
  explicit ADCCalibrationThread(rclcpp::Node::SharedPtr node);

  void run() override;

  void setNamespace(const std::string& ns);
  void setCurrentVoltage(double voltage);

private:
  const rclcpp::Node::SharedPtr node_;

  std::string ns_;
  double voltage_;

  size_t cnt_;
  algo::Kahan<double> voltage_sum_;

  void adcCb(const tobas_msgs::msg::Adc::ConstSharedPtr& adc);
};
}  // namespace hardware_setup
}  // namespace gui
