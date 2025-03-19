#pragma once

#include <rclcpp/node.hpp>

#include "tobas_setup_assistant/param_getters/double_table.hpp"
#include "./base.hpp"
#include "../propeller.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
/**
 * @brief 推力係数とトルク係数はThrust Standの実験データから求める．
 * 空気抗力係数はBlade Theoryから求める．
 */
class AerodynamicsWidget_ThrustStand : public AerodynamicsWidget_Base
{
  Q_OBJECT

  static constexpr int kTableHeight = 500;
  static constexpr int kTableColWidth = 180;

public:
  explicit AerodynamicsWidget_ThrustStand(rclcpp::Node::SharedPtr node, PropellerWidget* propeller);

  const char* name() const override;
  const char* description() const override;

  bool isValid() override;
  void copyFrom(const AerodynamicsWidget_Base* src) override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  double motorConst() const override;
  double momentConst() const override;
  double dragConst() const override;

private:
  const rclcpp::Node::SharedPtr node_;
  PropellerWidget* propeller_;
  ParamGetterWidget_DoubleTable* data_;
};
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
