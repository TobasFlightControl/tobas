#include <QVBoxLayout>

#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_qt_tools/util.hpp>

#include "tobas_simulation_gui/static_configuration/static_configuration.hpp"
#include "tobas_simulation_gui/constants.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace sim
{
StaticConfigWidget::StaticConfigWidget(rclcpp::Node::SharedPtr node)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  const auto title = new qt::Label("Static Configurations", kTitlePSize, QFont::Bold);
  qt::addWidgetCenter(title, rows);

  type_ = new SimulationTypeWidget();
  rows->addWidget(type_);

  world_ = new WorldWidget(node);
  rows->addWidget(world_);

  rows->addStretch();
}

sim_type_t StaticConfigWidget::simulationType() const
{
  return type_->simulationType();
}

fs::path StaticConfigWidget::worldPath() const
{
  return world_->worldPath();
}
}  // namespace sim
}  // namespace gui
