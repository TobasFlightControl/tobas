#pragma once

#include "./base_pose_commander.hpp"
#include "./joint_position_commander.hpp"

namespace gui
{
namespace sim
{
class CommandersWidget : public QWidget
{
  Q_OBJECT

  using self = CommandersWidget;
  using super = QWidget;

public:
  explicit CommandersWidget(rclcpp::Node::SharedPtr node, const kdl::Tree& tree, const tobas::Drone& drone);

  void updateInternalDataStructures();

  bool start();
  void reset();

private:
  BasePoseCommanderWidget* base_pose_commander_;
  JointPositionCommanderWidget* jointpos_commander_;
};
}  // namespace sim
}  // namespace gui
