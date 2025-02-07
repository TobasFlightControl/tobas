#include <QVBoxLayout>

#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_qt_tools/util.hpp>

#include "tobas_simulation_gui/commanders/commanders.hpp"
#include "tobas_simulation_gui/constants.hpp"

namespace gui
{
namespace sim
{
CommandersWidget::CommandersWidget(rclcpp::Node::SharedPtr node)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  const auto title = new qt::Label("Commanders", kTitlePSize, QFont::Bold);
  qt::addWidgetCenter(title, rows);

  base_pose_commander_ = new BasePoseCommanderWidget(node);
  rows->addWidget(base_pose_commander_);

  rows->addStretch();
}

bool CommandersWidget::start(const std::string& ns)
{
  if (!base_pose_commander_->start(ns))
    return false;

  return true;
}

void CommandersWidget::terminate()
{
  base_pose_commander_->terminate();
}
}  // namespace sim
}  // namespace gui
