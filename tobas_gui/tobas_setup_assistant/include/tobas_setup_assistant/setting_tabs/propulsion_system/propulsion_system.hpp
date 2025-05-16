#pragma once

#include <QButtonGroup>

#include <tobas_qt_tools/widgets/stacked_widget.hpp>

#include "../base_setting.hpp"
#include "./base.hpp"
#include "./electric/propulsion_system.hpp"
#include "./ice/propulsion_system.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
class PropulsionSystemWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = PropulsionSystemWidget;
  using super = BaseSettingWidget;

  static constexpr char kTypeKey[] = "propulsion_system_type";

  static constexpr int kElectricId = 0;
  static constexpr int kIceId = kElectricId + 1;

public:
  explicit PropulsionSystemWidget(rclcpp::Node::SharedPtr node, const RobotInfo& robot, Signals& _signals);

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void onOpened() override;
  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  tobas::propulsion_system_t type() const;
  int numUnits() const;

  QString linkName(int index) const;
  bool isTiltRotor(int index) const;
  QString tiltJointName(int index) const;

  BasePropulsionSystemWidget* widget(int index);
  const BasePropulsionSystemWidget* widget(int index) const;

  BasePropulsionSystemWidget* selected();
  const BasePropulsionSystemWidget* selected() const;

private:
  QButtonGroup* type_buttons_;
  qt::StackedWidget* propulsion_stack_;

private Q_SLOTS:
  void onPropulsionTypeChanged(int index);
};
};  // namespace propulsion
}  // namespace sa
}  // namespace gui
