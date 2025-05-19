#include "tobas_control_system/control_system.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>

namespace gui
{
namespace gcs
{
ControlSystemWidget::ControlSystemWidget(
  rclcpp::Node::SharedPtr node,
  const tobas::Drone& drone,
  const RosQtBridge& bridge)
  : drone_(drone)
{
  // Components
  pose_viewer_ = new PoseViewerWidget(node);
  power_source_viewer_ = new PowerSourceViewerWidget(node, drone);
  cpu_viewer_ = new CPUViewerWidget(bridge);
  rcin_viewer_ = new rcin::RCInputViewerWidget(node);
  rotors_viewer_ = new RotorsViewerWiddget(node, drone);
  console_ = new ConsoleWidget(node);
  status_viewer_ = new StatusViewerWidget(node);
  mission_planner_ = new MissionPlannerWidget(node, bridge);

  // Layout
  const auto cols3 = new QHBoxLayout();
  cols3->addWidget(power_source_viewer_, 1);
  cols3->addWidget(cpu_viewer_, 1);

  const auto rows2 = new QVBoxLayout();
  rows2->addLayout(cols3, 0);
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

void ControlSystemWidget::reset()
{
  pose_viewer_->reset();
  power_source_viewer_->reset();
  cpu_viewer_->reset();
  rcin_viewer_->reset();
  rotors_viewer_->reset();
  console_->reset();
  status_viewer_->reset();
  mission_planner_->reset();
}

void ControlSystemWidget::updateInternalDataStructures()
{
  pose_viewer_->updateNamespace(drone_.name);
  power_source_viewer_->updateInternalDataStructures();
  cpu_viewer_->reset();
  rcin_viewer_->updateNamespace(drone_.name);
  rotors_viewer_->updateInternalDataStructures();
  console_->updateNamespace(drone_.name);
  status_viewer_->updateNamespace(drone_.name);
  mission_planner_->updateNamespace(drone_.name);
}
}  // namespace gcs
}  // namespace gui
