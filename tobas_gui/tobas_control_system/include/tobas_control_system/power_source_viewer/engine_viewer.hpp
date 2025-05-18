#pragma once

#include <tobas_drone_core/drone.hpp>
#include <tobas_drone_core/propulsion_system/ice_propulsion_system/ice_propulsion_system.hpp>
#include <tobas_qt_tools/widgets/position_bar_widget.hpp>
#include <tobas_ros2_tools/register.hpp>

#include <tobas_msgs/msg/engine_state.hpp>

namespace gui
{
namespace gcs
{
class EngineViewerWidget : public QWidget
{
  Q_OBJECT

  using self = EngineViewerWidget;
  using super = QWidget;

  static constexpr int kLabelPSize = 12;
  static constexpr int kBarHeight = 40;

  static constexpr double kMinOilTemp = 0.;    // [degC]
  static constexpr double kMaxOilTemp = 130.;  // [degC]

Q_SIGNALS:
  void engineStateReceived(double fuel_quantity, double oil_temp);

public:
  explicit EngineViewerWidget(rclcpp::Node::SharedPtr node, const tobas::Drone& drone);

  void reset();
  void updateInternalDataStructures();

private:
  const rclcpp::Node::SharedPtr node_;
  const tobas::Drone& drone_;
  tobas::ICEPropulsionSystemConfig::ConstSharedPtr iprop_;

  qt::HPositionBarWidget* fuel_quantity_;
  qt::HPositionBarWidget* oil_temp_;

  ros2::SubscriberPtr<tobas_msgs::msg::EngineState> engine_state_sub_;

  void updateFuelQuantity(const double& fuel_quantity);
  void updateOilTemperature(const double& oil_temp);

  void engineStateCbRos(const tobas_msgs::msg::EngineState::ConstSharedPtr& engine_state);

private Q_SLOTS:
  void engineStateCbQt(double fuel_quantity, double oil_temp);
};
}  // namespace gcs
}  // namespace gui
