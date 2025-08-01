#pragma once

#include <yaml-cpp/yaml.h>

#include "tobas_setup_assistant/param_getters/double_table.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
class EngineDynamicsWidget : public QWidget
{
  Q_OBJECT

public:
  explicit EngineDynamicsWidget(rclcpp::Node::SharedPtr node);

  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  std::pair<double, double> engineConstant() const;

private:
  ParamGetterWidget_DoubleTable* data_;
};
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
