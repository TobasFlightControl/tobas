#pragma once

#include "./base.hpp"
#include "../../param_getters/line_edit.hpp"
#include "../../param_getters/combo_box.hpp"

namespace gui
{
namespace sa
{
class CustomControllerWidget : public BaseControllerWidget
{
  Q_OBJECT

  static constexpr char kRateThrottleLabel[] = "Euler Rate + Throttle";
  static constexpr char kAngleThrottleLabel[] = "Euler Angle + Throttle";
  static constexpr char kAccelYawLabel[] = "Accel + Yaw";
  static constexpr char kPosVelYawLabel[] = "Position + Velocity + Accel + Yaw";
  static constexpr char kPoseTwistAccelLabel[] = "Pose + Twist + Accel";
  static constexpr char kSpeedRollDeltaPitchLabel[] = "Speed + Roll + Pitch";

public:
  explicit CustomControllerWidget();

  const char* name() const override;
  const char* description() const override;
  QString controllerPackage() const override;
  QString pluginName() const override;

  tobas::rc_command_t acrobatModeCommand() const override;
  tobas::rc_command_t stabilizeModeCommand() const override;
  tobas::rc_command_t loiterModeCommand() const override;

  YAML::Node staticParams() const override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  bool isApplicable() override;
  bool isValid() override;

private:
  const std::map<QString, tobas::rc_command_t> command_map_;

  ParamGetterWidget_LineEdit* package_;
  ParamGetterWidget_LineEdit* plugin_;
  ParamGetterWidget_ComboBox* acrobat_mode_;
  ParamGetterWidget_ComboBox* stabilize_mode_;
  ParamGetterWidget_ComboBox* loiter_mode_;
};
}  // namespace sa
}  // namespace gui
