#pragma once

#include <yaml-cpp/yaml.h>

#include <tobas_drone_core/propulsion_system/ice_propulsion_system/aerodynamics.hpp>

#include "tobas_setup_assistant/param_getters/double_table.hpp"

#include "./base.hpp"
#include "./propeller.hpp"

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
class AerodynamicsWidget : public BaseSelectedLinkSettingWidget
{
  Q_OBJECT

public:
  explicit AerodynamicsWidget(rclcpp::Node::SharedPtr node, const PropellerWidget* propeller);

  const char* name() const override;
  bool isValid() override;
  void copyFrom(const BaseSelectedLinkSettingWidget* src) override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  VppMotorConstant motorConst() const;
  VppMomentConstant momentConst() const;
  VppDragConstant dragConst() const;

private:
  const PropellerWidget* const propeller_;

  ParamGetterWidget_DoubleTable* data_;

  std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> getData() const;
};
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
