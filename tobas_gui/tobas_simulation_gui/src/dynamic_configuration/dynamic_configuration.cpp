#include <QVBoxLayout>

#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_qt_tools/util.hpp>

#include "tobas_simulation_gui/dynamic_configuration/dynamic_configuration.hpp"
#include "tobas_simulation_gui/constants.hpp"

namespace gui
{
namespace sim
{
DynamicConfigWidget::DynamicConfigWidget(rclcpp::Node::SharedPtr node)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  const auto title = new qt::Label("Dynamic Configurations", kTitlePSize, QFont::Bold);
  qt::addWidgetCenter(title, rows);

  wind_ = new WindParamsWidget(node);
  rows->addWidget(wind_);

  rows->addStretch();
}

bool DynamicConfigWidget::initialize(const std::string& ns)
{
  if (!wind_->initialize(ns))
    return false;

  return true;
}
}  // namespace sim
}  // namespace gui
