#pragma once

#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/widgets/stacked_widget.hpp>

#include "./base.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
class EngineDynamicsWidget : public QWidget
{
  Q_OBJECT

  static constexpr char kMethodNameKey[] = "method_name";

public:
  explicit EngineDynamicsWidget();

  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  /* [Nm/(rad/s)] */
  double torqueConstant() const;

  /* [Nm] */
  double dynamicFrictionTorque() const;

private:
  qt::ComboBox* method_name_;
  qt::StackedWidget* methods_;

  EngineDynamicsWidget_Base* selected();
  const EngineDynamicsWidget_Base* selected() const;
};
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
