#include "tobas_simulation_gui/commanders/commanders.hpp"

#include <QVBoxLayout>

#include <tobas_qt_tools/util.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

#include "tobas_simulation_gui/constants.hpp"

namespace gui
{
namespace sim
{
CommandersWidget::CommandersWidget(rclcpp::Node::SharedPtr node, const kdl::Tree& tree, const tobas::Drone& drone)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  const auto title = new qt::Label("Commanders", kTitlePSize, QFont::Bold);
  qt::addWidgetCenter(title, rows);

  const auto scroll_rows = qt::createScrollableQVBoxLayout(rows);

  base_pose_commander_ = new BasePoseCommanderWidget(node, drone);
  scroll_rows->addWidget(base_pose_commander_);

  jointpos_commander_ = new JointPositionCommanderWidget(node, tree, drone);
  scroll_rows->addWidget(jointpos_commander_);

  scroll_rows->addStretch();
}

void CommandersWidget::updateInternalDataStructures()
{
  base_pose_commander_->updateInternalDataStructures();
  jointpos_commander_->updateInternalDataStructures();
}

bool CommandersWidget::start()
{
  if (!base_pose_commander_->start()) {
    return false;
  }

  if (!jointpos_commander_->start()) {
    return false;
  }

  return true;
}

void CommandersWidget::reset()
{
  base_pose_commander_->reset();
  jointpos_commander_->reset();
}
}  // namespace sim
}  // namespace gui
