#include <tobas_yaml_tools/convert/qstring.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/engine/response.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
EngineResponseWidget::EngineResponseWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  time_const_up_ = new ParamGetterWidget_SpinBox("Time Constant Up", "");  // TODO
  time_const_up_->setMinimum(0);
  time_const_up_->setValue(100);
  time_const_up_->setSuffix(" ms");
  rows->addWidget(time_const_up_);

  time_const_down_ = new ParamGetterWidget_SpinBox("Time Constant Down", "");  // TODO
  time_const_down_->setMinimum(0);
  time_const_down_->setValue(200);
  time_const_down_->setSuffix(" ms");
  rows->addWidget(time_const_down_);

  rows->addStretch();
}

bool EngineResponseWidget::isValid()
{
  return true;
}

YAML::Node EngineResponseWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[time_const_up_->name()] = time_const_up_->getValue();
  node[time_const_down_->name()] = time_const_down_->getValue();

  return node;
}

void EngineResponseWidget::load(const YAML::Node& node)
{
  time_const_up_->setValue(node[time_const_up_->name()].as<double>());
  time_const_down_->setValue(node[time_const_down_->name()].as<double>());
}

double EngineResponseWidget::timeConstUp() const
{
  return time_const_up_->getValue() * 1e-3;
}

double EngineResponseWidget::timeConstDown() const
{
  return time_const_down_->getValue() * 1e-3;
}
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
