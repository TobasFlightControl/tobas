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
class EngineHardwareIfaceWidget : public QWidget
{
  Q_OBJECT

  using self = EngineHardwareIfaceWidget;
  using super = QWidget;

public:
  explicit EngineHardwareIfaceWidget();

  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  int pwmChannel() const;

  /* [us] */
  int pwmPeriodZeroThrot() const;

  /* [us] */
  int pwmPeriodFullThrot() const;

private:
  ParamGetterWidget_SpinBox* pwm_channel_;
  ParamGetterWidget_SpinBox* pwm_period_zero_;
  ParamGetterWidget_SpinBox* pwm_period_full_;
};
};  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
