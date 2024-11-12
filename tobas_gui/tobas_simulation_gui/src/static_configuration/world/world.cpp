#include <QButtonGroup>

#include <tobas_qt_tools/widgets/label.hpp>

#include "tobas_simulation_gui/constants.hpp"
#include "tobas_simulation_gui/static_configuration/world/world.hpp"
#include "tobas_simulation_gui/static_configuration/world/standard_world.hpp"
#include "tobas_simulation_gui/static_configuration/world/custom_world.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace sim
{
WorldWidget::WorldWidget(rclcpp::Node::SharedPtr node)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  const auto label = new qt::Label("World", kLabelPSize, QFont::Bold);
  rows->addWidget(label);

  widgets_.push_back(new WorldWidget_Standard());
  widgets_.push_back(new WorldWidget_Custom(node));

  const auto ckb_group = new QButtonGroup(this);
  ckb_group->setExclusive(true);

  for (const auto& widget : widgets_)
  {
    rows->addWidget(widget);
    ckb_group->addButton(widget->checkbox);
  }

  // Default
  widgets_.at(0)->checkbox->setChecked(true);
}

fs::path WorldWidget::worldPath() const
{
  return selected()->worldPath();
}

const WorldWidget_Base* WorldWidget::selected() const
{
  for (const auto& widget : widgets_)
    if (widget->isChecked())
      return widget;

  throw std::runtime_error("No method is selected.");
}
}  // namespace sim
}  // namespace gui
