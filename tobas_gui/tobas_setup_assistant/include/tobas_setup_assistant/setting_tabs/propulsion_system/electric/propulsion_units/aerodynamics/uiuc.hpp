#pragma once

#include "../propeller.hpp"
#include "./base.hpp"
#include "tobas_setup_assistant/param_getters/double_table.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
class AerodynamicsWidget_UIUC : public AerodynamicsWidget_Base
{
  Q_OBJECT

public:
  explicit AerodynamicsWidget_UIUC(rclcpp::Node::SharedPtr node, const PropellerWidget* propeller);

  const char* name() const override;
  const char* description() const override;

  bool isValid() override;
  void copyFrom(const AerodynamicsWidget_Base* src) override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  double motorConst() const override;
  double momentConst() const override;

private:
  const PropellerWidget* const propeller_;

  ParamGetterWidget_DoubleTable* data_;
};
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
