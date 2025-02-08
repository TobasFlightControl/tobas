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

  wind_ = new WindParamsWidget(node);
  scroll_rows->addWidget(wind_);

  scroll_rows->addStretch();
}

bool DynamicConfigWidget::start(const std::string& ns)
{
  if (!wind_->start(ns))
    return false;

  return true;
}

void DynamicConfigWidget::terminate()
{
  wind_->terminate();
}
}  // namespace sim
}  // namespace gui
