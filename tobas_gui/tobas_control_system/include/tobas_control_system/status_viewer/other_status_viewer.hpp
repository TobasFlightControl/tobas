#pragma once

#include <tobas_rqt_bridge/bridge.hpp>

#include "./status.hpp"

namespace gui
{
namespace gcs
{
class OtherStatusViewerWidget : public QWidget
{
  Q_OBJECT

  using self = OtherStatusViewerWidget;
  using super = QWidget;

public:
  explicit OtherStatusViewerWidget(const RosQtBridge& bridge);

  void reset();

private:
  StatusWidget* arming_status_;
  StatusWidget* gnss_status_;

private Q_SLOTS:
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void gnssCb(const tobas_msgs::Gnss::ConstSharedPtr& gnss);
};
}  // namespace gcs
}  // namespace gui
