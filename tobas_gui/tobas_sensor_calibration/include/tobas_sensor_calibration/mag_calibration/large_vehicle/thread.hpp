#pragma once

#include <QThread>

#include <tobas_algorithm/kahan.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

namespace gui
{
namespace sc
{
class LargeVehicleMagCalibThread : public QThread
{
  Q_OBJECT

  using self = LargeVehicleMagCalibThread;
  using super = QThread;

  static constexpr size_t kDataCount = 200;
  static constexpr auto kCollectDataTimeout = std::chrono::seconds(10);

Q_SIGNALS:
  void finished(bool success, const QString& message);

public:
  explicit LargeVehicleMagCalibThread(rclcpp::Node::SharedPtr node, const RosQtBridge& bridge);

  void run() override;

  void reset();
  void setNamespace(const std::string& ns);

private:
  const rclcpp::Node::SharedPtr node_;

  std::string ns_;

  tobas_msgs::MagneticField::ConstSharedPtr mag_raw_;
  tobas_msgs::Gnss::ConstSharedPtr gnss_;
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

  bool get_data_ = false;
  size_t cnt_;
  std::array<algo::Kahan<double>, 3> mag_sum_;

private Q_SLOTS:
  void magCb(const tobas_msgs::MagneticField::ConstSharedPtr& mag_raw);
  void gnssCb(const tobas_msgs::Gnss::ConstSharedPtr& gnss);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
};
}  // namespace sc
}  // namespace gui
