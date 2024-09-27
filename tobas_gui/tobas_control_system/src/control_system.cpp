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
  battery_cpu_viewer_ = new BatteryCPUViewerWidget(node, drone);
  rcin_viewer_ = new rcin::RCInputViewerWidget(node);
  mission_planner_ = new MissionPlannerWidget(node);
  // TODO

  // Layout
  const auto rows2 = new QVBoxLayout();
  rows2->addWidget(battery_cpu_viewer_);
  rows2->addWidget(rcin_viewer_);
  // TODO: Rotors Viewer

  const auto cols2 = new QHBoxLayout();
  cols2->addLayout(rows2);
  // TODO: Status Viewer

  const auto rows1 = new QVBoxLayout();
  rows1->addWidget(pose_viewer_, 1);
  rows1->addLayout(cols2, 1);

  const auto cols1 = new QHBoxLayout();
  cols1->addLayout(rows1, 1);
  cols1->addWidget(mission_planner_, 1);

  setLayout(cols1);
}

void ControlSystemWidget::updateInternalDataStructures()
{
  pose_viewer_->updateNamespace(drone_.name);
  battery_cpu_viewer_->updateInternalDataStructures();
  rcin_viewer_->updateNamespace(drone_.name);
  mission_planner_->updateNamespace(drone_.name);
}
}  // namespace control_system
}  // namespace gui
