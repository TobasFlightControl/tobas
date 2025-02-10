#pragma once

#include <rclcpp/node.hpp>

#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/widgets/stacked_widget.hpp>

#include "./base.hpp"
#include "../base.hpp"
#include "../propeller.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
class AerodynamicsWidget : public BaseSelectedLinkSettingWidget
{
  Q_OBJECT

  static constexpr char kMethodNameKey[] = "method_name";

public:
  explicit AerodynamicsWidget(rclcpp::Node::SharedPtr node, PropellerWidget* propeller);

  const char* name() const override;
  bool isValid() override;
  void copyFrom(const BaseSelectedLinkSettingWidget* src) override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  /* [kg*m/rad^2] */
  double motorConst() const;

  /* [m] */
  double momentConst() const;

  /* [kg/rad] */
  double rotorDragCoef() const;

private:
  qt::ComboBox* method_name_;
  qt::StackedWidget* methods_;

  AerodynamicsWidget_Base* selected();
  const AerodynamicsWidget_Base* selected() const;
};
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
