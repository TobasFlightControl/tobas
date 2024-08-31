#pragma once

#include <rclcpp/node.hpp>

#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/widgets/stacked_widget.hpp>

#include "./base.hpp"
#include "../base.hpp"
#include "../motor.hpp"
#include "../aerodynamics/aerodynamics.hpp"

namespace gui
{
namespace setup_assistant
{
class ElectrodynamicsWidget : public BaseSelectedLinkSettingWidget<ElectrodynamicsWidget>
{
  Q_OBJECT

  static constexpr char kMethodNameKey[] = "method_name";

public:
  explicit ElectrodynamicsWidget(rclcpp::Node::SharedPtr node, MotorWidget* motor, AerodynamicsWidget* aerodynamics);

  const char* name() override;
  bool isValid() override;
  void copyFrom(const ElectrodynamicsWidget* src) override;

  YAML::Node dump() override;
  void load(const YAML::Node& node) override;

  /* V = a w + b w^2 (V[V], w[rad/s]) */
  std::pair<double, double> rotSpeedCoefs() const;

private:
  qt::ComboBox* method_name_;
  qt::StackedWidget* methods_;

  ElectrodynamicsWidget_Base* selected();
  const ElectrodynamicsWidget_Base* selected() const;
};
}  // namespace setup_assistant
}  // namespace gui
