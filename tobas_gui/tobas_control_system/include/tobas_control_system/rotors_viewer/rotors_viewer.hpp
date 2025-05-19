#pragma once

#include <QHBoxLayout>

#include <tobas_drone_core/drone.hpp>
#include <tobas_qt_tools/widgets/scroll_area.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

#include <tobas_msgs/msg/rotor_liveliness_array.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>

#include "./speedmeter.hpp"

namespace gui
{
namespace gcs
{
class RotorsViewerWiddget : public qt::ScrollArea
{
  Q_OBJECT

  using self = RotorsViewerWiddget;
  using super = qt::ScrollArea;

  static constexpr char kAliveBackgroundColor[] = "transparent";
  static constexpr char kDeadBackgroundColor[] = "red";

public:
  explicit RotorsViewerWiddget(const RosQtBridge& bridge, const tobas::Drone& drone);

  void reset();
  void updateInternalDataStructures();

private:
  const tobas::Drone& drone_;

  std::map<std::string, SpeedmeterWidget*> meters_;
  QHBoxLayout* cols_;

  void setSpeed(const std::string& link_name, const double& rps);

  static QString bottomText(int rpm);

private Q_SLOTS:
  void rotorStatesCb(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& msg);
  void rotorLivelinessCb(const tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr& msg);
};
}  // namespace gcs
}  // namespace gui
