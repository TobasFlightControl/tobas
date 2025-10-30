#pragma once

#include <QThread>

#include <tobas_algorithm/kahan.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

namespace gui
{
namespace sc
{
class AccelCalibrationThread : public QThread
{
  Q_OBJECT

  using self = AccelCalibrationThread;
  using super = QThread;

  static constexpr size_t kDataCount = 200;
  static constexpr double kCollectDataTimeout = 10.;  // [s]
  static constexpr double kAccelBiasNormThresh = 1.;  // [m/s^2] ISM330DLCで最大 85mg (= 0.83m/s^2) のオフセット

Q_SIGNALS:
  void finished(bool success, const QString& message);

public:
  explicit AccelCalibrationThread(rclcpp::Node::SharedPtr node, const RosQtBridge& bridge);

  void run() override;

  void reset();
  void setNamespace(const std::string& ns);

private:
  const rclcpp::Node::SharedPtr node_;

  std::string ns_;

  tobas_msgs::Imu::ConstSharedPtr imu_raw_;

  bool get_data_ = false;
  size_t cnt_;
  std::array<algo::Kahan<double>, 3> acc_sum_;

private Q_SLOTS:
  void imuCb(const tobas_msgs::Imu::ConstSharedPtr& imu_raw);
};
}  // namespace sc
}  // namespace gui
