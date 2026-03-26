#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/propulsion_system.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
PropulsionSystemWidget::PropulsionSystemWidget(rclcpp::Node::SharedPtr node, const uadf::Model& uadf)
{
  engine = new EngineWidget(node);
  units = new PropulsionUnitsWidget(node, uadf);

  // Layout
  const auto battery_rows = new QVBoxLayout();
  tobas::qt::addWidgetCenter(new tobas::qt::Label("Engine", cmn::kTitlePSize, QFont::Bold), battery_rows);
  battery_rows->addWidget(engine);

  const auto units_rows = new QVBoxLayout();
  tobas::qt::addWidgetCenter(new tobas::qt::Label("Propulsion Units", cmn::kTitlePSize, QFont::Bold), units_rows);
  units_rows->addWidget(units);

  const auto cols = new QHBoxLayout();
  cols->addLayout(battery_rows, 1);
  cols->addLayout(units_rows, 1);

  setLayout(cols);
}

const char* PropulsionSystemWidget::name() const
{
  return "ICE Propulsion System";
}

void PropulsionSystemWidget::updateInternalDataStructures()
{
  units->updateInternalDataStructures();
}

bool PropulsionSystemWidget::isValid()
{
  if (!engine->isValid()) {
    return false;
  }

  if (!units->isValid()) {
    return false;
  }

  return true;
}

YAML::Node PropulsionSystemWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kEngineTitle] = engine->dump();
  node[kPropulsionUnitTitle] = units->dump();

  return node;
}

void PropulsionSystemWidget::load(const YAML::Node& node)
{
  engine->load(node[kEngineTitle]);
  units->load(node[kPropulsionUnitTitle]);
}

tobas::PropulsionSystem PropulsionSystemWidget::type() const
{
  return tobas::PropulsionSystem::kIce;
}

int PropulsionSystemWidget::numUnits() const
{
  return units->count();
}

QString PropulsionSystemWidget::linkName(int index) const
{
  return units->linkName(index);
}
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
