#pragma once

#include "../base.hpp"
#include "./engine/engine.hpp"
#include "./propulsion_units/propulsion_units.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
class PropulsionSystemWidget : public BasePropulsionSystemWidget
{
  Q_OBJECT

  using self = PropulsionSystemWidget;
  using super = BasePropulsionSystemWidget;

  static constexpr char kEngineTitle[] = "Engine";
  static constexpr char kPropulsionUnitTitle[] = "Propulsion Units";

public:
  EngineWidget* engine;
  PropulsionUnitsWidget* units;

  explicit PropulsionSystemWidget(rclcpp::Node::SharedPtr node, const RobotInfo& robot, Signals& _signals);

  const char* name() const override;

  void reset() override;
  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  tobas::propulsion_system_t type() const override;
  int numUnits() const override;

  QString linkName(int index) const override;
  bool isTiltRotor(int index) const override;
  QString tiltJointName(int index) const override;
};
};  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
