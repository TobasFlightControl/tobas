#include <QVBoxLayout>
#include <QHBoxLayout>

#include "tobas_control_system/control_system.hpp"

namespace gui
{
namespace control_system
{
ControlSystemWidget::ControlSystemWidget(rclcpp::Node::SharedPtr node, const tobas::Drone& drone)
  : node_(node), drone_(drone)
{
  // Components
  pose_viewer_ = new PoseViewerWidget(node);
  mission_planner_ = new MissionPlannerWidget(node);
  // TODO

  // Layout
  const auto cols1 = new QHBoxLayout();
  const auto rows1 = new QVBoxLayout();

  rows1->addWidget(pose_viewer_);

  setLayout(cols1);
  cols1->addLayout(rows1, 1);
  // TODO
  cols1->addWidget(mission_planner_, 1);
}

void ControlSystemWidget::updateInternalDataStructures()
{
  pose_viewer_->updateNamespace(drone_.name);
  mission_planner_->updateNamespace(drone_.name);
}
}  // namespace control_system
}  // namespace gui
