#pragma once

#include <tobas_drone_core/propulsion_system/turning_direction.hpp>

#include "tobas_setup_assistant/param_getters/spin_box.hpp"
#include "tobas_setup_assistant/param_getters/combo_box.hpp"
#include "../base.hpp"
#include "./active_tilt_settings.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
class GeneralWidget : public BaseSelectedLinkSettingWidget
{
  Q_OBJECT

  using self = GeneralWidget;
  using super = BaseSelectedLinkSettingWidget;

  static constexpr char kCWName[] = "CW";
  static constexpr char kCCWName[] = "CCW";
  static constexpr char kActiveTiltSettingsKey[] = "active_tilt_settings";

Q_SIGNALS:
  void channelChanged(int channel);
  void isTiltStateChanged(bool is_tilt);
  void tiltJointNameChanged(const QString& joint_name);

public:
  explicit GeneralWidget(const RobotInfo& robot, const QString& link_name);

  const char* name() const override;
  bool isValid() override;
  void copyFrom(const BaseSelectedLinkSettingWidget* src) override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  int channel() const;
  tobas::turning_direction_t direction() const;
  bool isTiltRotor() const;
  QString tiltJointName() const;

private:
  ParamGetterWidget_SpinBox* channel_;
  ParamGetterWidget_ComboBox* direction_;
  ActiveTiltSettingsWidget* active_tilt_settings_;
};
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
