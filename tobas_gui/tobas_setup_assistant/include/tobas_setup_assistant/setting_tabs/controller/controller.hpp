#pragma once

#include <tobas_qt_tools/widgets/stacked_widget.hpp>

#include "../base_setting.hpp"
#include "./active_tilt_multicopter.hpp"
#include "./fixed_wing.hpp"
#include "./non_planar_multicopter.hpp"
#include "./planar_multicopter.hpp"
#include "tobas_setup_assistant/frame_type.hpp"

namespace gui
{
namespace sa
{
namespace ctrl
{
class ControllerWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = ControllerWidget;
  using super = BaseSettingWidget;

public:
  explicit ControllerWidget();

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  void setFrameType(const FrameType& type);

  QString controllerPackage() const;
  QString pluginName() const;

  tobas::RcCommand acrobatModeCommand() const;
  tobas::RcCommand stabilizeModeCommand() const;
  tobas::RcCommand loiterModeCommand() const;

  YAML::Node staticParams() const;

  bool isCommandCompatible(tobas::RcCommand command) const;

private Q_SLOTS:
  void setCurrentController(int index);

private:
  qt::StackedWidget* stack_;

  PlanarMulticopterWidget* planar_multicopter_;
  NonPlanarMulticopterWidget* nonplanar_multicopter_;
  ActiveTiltMulticopterWidget* active_tilt_multicopter_;
  FixedWingWidget* fixed_wing_;

  BaseControllerWidget* widget(int index);
  const BaseControllerWidget* widget(int index) const;

  BaseControllerWidget* selected();
  const BaseControllerWidget* selected() const;
};
}  // namespace ctrl
};  // namespace sa
}  // namespace gui
