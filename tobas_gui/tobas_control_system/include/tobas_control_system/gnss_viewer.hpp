#pragma once

#include <tobas_qt_tools/widgets/framed_label.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

namespace gui
{
namespace ctrl
{
class GnssViewerWidget : public QWidget
{
  Q_OBJECT

  using self = GnssViewerWidget;
  using super = QWidget;

  static constexpr int kLabelPSize = 12;

public:
  explicit GnssViewerWidget(const RosQtBridge& bridge);

  void reset();

private:
  qt::FramedLabel* status_;
  qt::FramedLabel* nsats_;

  void setStatus(uint8_t status);
  void setNSats(int nsats);

private Q_SLOTS:
  void gnssCb(const tobas_msgs::Gnss::ConstSharedPtr& gnss);
};
}  // namespace ctrl
}  // namespace gui
