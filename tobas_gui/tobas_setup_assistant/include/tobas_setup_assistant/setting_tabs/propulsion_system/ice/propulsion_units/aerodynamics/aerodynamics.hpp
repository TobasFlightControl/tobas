#pragma once

#include <rclcpp/node.hpp>

#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/widgets/stacked_widget.hpp>

#include "../base.hpp"
#include "../propeller.hpp"
#include "./base.hpp"

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

  static constexpr char kMethodNameKey[] = "method_name";

public:
  explicit AerodynamicsWidget();

  const char* name() const override;
  bool isValid() override;
  void copyFrom(const BaseSelectedLinkSettingWidget* src) override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  /* [kg*m/rad^3, kg*m/rad^2] */
  std::pair<double, double> motorConst() const;

  /* [m] */
  double momentConst() const;

  /* [kg/rad^2, kg/rad] */
  std::pair<double, double> dragConst() const;

private:
  qt::ComboBox* method_name_;
  qt::StackedWidget* methods_;

  AerodynamicsWidget_Base* selected();
  const AerodynamicsWidget_Base* selected() const;
};
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
