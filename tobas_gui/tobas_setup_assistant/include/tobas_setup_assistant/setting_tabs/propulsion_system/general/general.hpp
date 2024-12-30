#pragma once

#include "tobas_setup_assistant/param_getters/spin_box.hpp"
#include "../base.hpp"
#include "./active_tilt_settings.hpp"

namespace gui
{
namespace setup_assistant
{
namespace propulsion_system
{
class GeneralWidget : public BaseSelectedLinkSettingWidget
{
  Q_OBJECT

  static constexpr char kActiveTiltSettingsKey[] = "active_tilt_settings";

public:
  explicit GeneralWidget();

  const char* name() const override;
  bool isValid() override;
  void copyFrom(const BaseSelectedLinkSettingWidget* src) override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  int channel() const;
  bool isTiltRotor() const;
  QString tiltJointName() const;

private:
  ParamGetterWidget_SpinBox* channel_;
  ActiveTiltSettingsWidget* active_tilt_settings_;
};
}  // namespace propulsion_system
}  // namespace setup_assistant
}  // namespace gui
