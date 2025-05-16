#pragma once

#include <QThread>
#include <eigen3/Eigen/Core>
#include <rclcpp/node.hpp>

#include <tobas_algorithm/kahan.hpp>

#include <tobas_msgs_adapter/imu_stamped.hpp>

namespace gui
{
namespace hw
{
class AccelCalibrationThread : public QThread
{
  Q_OBJECT

  using self = AccelCalibrationThread;
  using super = QThread;

  static constexpr size_t kDataCount = 200;
  static constexpr double kCollectDataTimeout = 10.;  // [s]

Q_SIGNALS:
  void finished(bool success, const QString& message);

public:
  explicit AccelCalibrationThread(rclcpp::Node::SharedPtr node);

  void run() override;

  void setNamespace(const std::string& ns);

private:
  const rclcpp::Node::SharedPtr node_;

  std::string ns_;

  size_t cnt_;
  std::array<algo::Kahan<double>, 3> acc_sum_;
  Eigen::Vector3d acc_top_;

  bool getAccelMean(Eigen::Vector3d& des);
  void imuCb(const tobas_msgs::ImuStamped::ConstSharedPtr& imu_raw);
};
}  // namespace hw
}  // namespace gui
