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

  const auto scroll_rows = qt::createScrollableQVBoxLayout(rows);

  wind_params_ = new WindParamsWidget(node);
  scroll_rows->addWidget(wind_params_);

  scroll_rows->addStretch();
}

void DynamicConfigWidget::updateNamespace(const std::string& ns)
{
  wind_params_->updateNamespace(ns);
}

bool DynamicConfigWidget::start()
{
  if (!wind_params_->start())
    return false;

  return true;
}

void DynamicConfigWidget::reset()
{
  wind_params_->reset();
}
}  // namespace sim
}  // namespace gui
