#include <QVBoxLayout>

#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_qt_tools/util.hpp>

#include "tobas_simulation_gui/simulation_settings/simulation_settings.hpp"
#include "tobas_simulation_gui/constants.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace sim
{
SimulationSettingsWidget::SimulationSettingsWidget(rclcpp::Node::SharedPtr node)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  const auto title = new qt::Label("Simulation Settings", kTitlePSize, QFont::Bold);
  qt::addWidgetCenter(title, rows);

  type_ = new LoopTypeWidget();
  rows->addWidget(type_);

  world_ = new WorldWidget(node);
  rows->addWidget(world_);
}

loop_type_t SimulationSettingsWidget::loopType() const
{
  return type_->loopType();
}

fs::path SimulationSettingsWidget::worldPath() const
{
  return world_->worldPath();
}
}  // namespace sim
}  // namespace gui
