#pragma once

#include <tobas_rqt_bridge/bridge.hpp>

#include "./status.hpp"

namespace gui
{
namespace gcs
{
class PostArmCheckViewerWidget : public QWidget
{
  Q_OBJECT

  using self = PostArmCheckViewerWidget;
  using super = QWidget;

public:
  explicit PostArmCheckViewerWidget(const RosQtBridge& bridge);

  void reset();

private:
  StatusWidget* gyro_noise_status_;
  StatusWidget* accel_noise_status_;
  StatusWidget* mag_offset_status_;
  StatusWidget* mag_alignment_status_;
  StatusWidget* latency_status_;

  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

private Q_SLOTS:
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void postArmCheckCb(const tobas_msgs::msg::PostArmCheck::ConstSharedPtr& postarm_check);
};
}  // namespace gcs
}  // namespace gui
