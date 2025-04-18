#pragma once

#include <yaml-cpp/yaml.h>

#include "tobas_setup_assistant/param_getters/spin_box.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
class EngineLimitWidget : public QWidget
{
  Q_OBJECT

  using self = EngineLimitWidget;
  using super = QWidget;

public:
  explicit EngineLimitWidget();

  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  /* [rad/s] */
  double maxSpeed() const;

private:
  ParamGetterWidget_SpinBox* max_speed_;
};
};  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
