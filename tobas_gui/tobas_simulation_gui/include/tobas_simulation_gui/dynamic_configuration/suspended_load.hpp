#pragma once

#include <QPushButton>

#include <tobas_qt_tools/widgets/double_spin_box.hpp>
#include <tobas_qt_tools/widgets/spin_box.hpp>
#include <tobas_qt_tools/widgets/toggle_button.hpp>
#include <tobas_qt_tools/widgets/vector3d_edit_vertical.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>

#include <tobas_gazebo_msgs/srv/attach_suspended_load.hpp>
#include <tobas_gazebo_msgs/srv/detach_suspended_load.hpp>

namespace tobas
{
namespace gui
{
namespace sim
{
class SuspendedLoadWidget : public QWidget
{
  Q_OBJECT

  using self = SuspendedLoadWidget;
  using super = QWidget;
  using AttachSrv = tobas_gazebo_msgs::srv::AttachSuspendedLoad;
  using DetachSrv = tobas_gazebo_msgs::srv::DetachSuspendedLoad;

  static constexpr double kDefaultLoadSize = 0.3;           // [m]
  static constexpr double kDefaultLoadMass = 1.;            // [kg]
  static constexpr double kDefaultCableLength = 3.;         // [m]
  static constexpr int kDefaultCableYoungModulus = 200;     // [MPa] 低密度ポリエチレン
  static constexpr int kDefaultCableCrossSectionArea = 50;  // [mm^2]

public:
  explicit SuspendedLoadWidget(rclcpp::Node::SharedPtr node);

  void updateNamespace(const std::string& ns);

  bool start();
  void reset();

private:
  const rclcpp::Node::SharedPtr node_;
  ros2::SyncServiceClient<AttachSrv>::SharedPtr attach_sc_;
  ros2::SyncServiceClient<DetachSrv>::SharedPtr detach_sc_;

  tobas::qt::ToggleButton* attach_detach_btn_;

  tobas::qt::Vector3dEditVertical* attach_point_;
  tobas::qt::Vector3dEditVertical* load_size_;
  tobas::qt::DoubleSpinBox* load_mass_;
  tobas::qt::DoubleSpinBox* cable_length_;
  tobas::qt::SpinBox* cable_young_;
  tobas::qt::SpinBox* cable_csa_;

  void setParamsToDefault();

private Q_SLOTS:
  void onAttachRequested();
  void onDetachRequested();
};
}  // namespace sim
}  // namespace gui
}  // namespace tobas
