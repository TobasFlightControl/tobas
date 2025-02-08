#include <QVBoxLayout>

#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_qt_tools/util.hpp>

#include "tobas_simulation_gui/commanders/commanders.hpp"
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

  base_pose_commander_ = new BasePoseCommanderWidget(node, drone);
  rows->addWidget(base_pose_commander_);

  jointpos_commander_ = new JointPositionCommanderWidget(node, tree, drone);
  rows->addWidget(jointpos_commander_);

  rows->addStretch();
}

bool CommandersWidget::start()
{
  if (!base_pose_commander_->start())
    return false;

  if (!jointpos_commander_->start())
    return false;

  return true;
}

void CommandersWidget::terminate()
{
  base_pose_commander_->terminate();
  jointpos_commander_->terminate();
}
}  // namespace sim
}  // namespace gui
