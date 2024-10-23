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
  rotors_viewer_ = new RotorsViewerWiddget(node, drone);
  console_ = new ConsoleWidget(node);
  status_viewer_ = new StatusViewerWidget(node);
  mission_planner_ = new MissionPlannerWidget(node);

  // Layout
  const auto rows2 = new QVBoxLayout();
  rows2->addWidget(battery_cpu_viewer_, 0);
  rows2->addWidget(rcin_viewer_, 1);
  rows2->addWidget(rotors_viewer_, 1);
  rows2->addWidget(console_, 1);

  const auto cols2 = new QHBoxLayout();
  cols2->addLayout(rows2, 3);
  cols2->addWidget(status_viewer_, 1);

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
  rotors_viewer_->updateInternalDataStructures();
  console_->updateNamespace(drone_.name);
  status_viewer_->updateNamespace(drone_.name);
  mission_planner_->updateNamespace(drone_.name);
}
}  // namespace control_system
}  // namespace gui
