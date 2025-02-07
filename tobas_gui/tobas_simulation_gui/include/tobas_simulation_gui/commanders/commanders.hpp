#pragma once

#include "./base_pose_commander.hpp"

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
  explicit CommandersWidget(rclcpp::Node::SharedPtr node);

  bool start(const std::string& ns);
  void terminate();

private:
  BasePoseCommanderWidget* base_pose_commander_;
};
}  // namespace sim
}  // namespace gui
