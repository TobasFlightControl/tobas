#pragma once

#include "../../param_getters/combo_box.hpp"
#include "../../param_getters/line_edit.hpp"
#include "./base.hpp"

namespace gui
{
namespace sa
{
class CustomControllerWidget : public BaseControllerWidget
{
  Q_OBJECT

  static constexpr char kRateThrottleLabel[] = "Angle Rate + Throttle";
  static constexpr char kAngleThrottleLabel[] = "Euler Angle + Throttle";
  static constexpr char kAccelYawLabel[] = "Accel + Yaw";
  static constexpr char kPosVelYawLabel[] = "Position + Velocity + Yaw";
  static constexpr char kAccelRateLabel[] = "Accel + Angle Rate";
  static constexpr char kAccelAngleLabel[] = "Accel + Euler Angle";
  static constexpr char kPosVelAngleLabel[] = "Position + Velocity + Angle";
  static constexpr char kSpeedRollDeltaPitchLabel[] = "Speed + Roll + Pitch";

public:
  explicit CustomControllerWidget();

  const char* name() const override;
  QString controllerPackage() const override;
  QString pluginName() const override;

  tobas::RcCommand acrobatModeCommand() const override;
  tobas::RcCommand stabilizeModeCommand() const override;
  tobas::RcCommand loiterModeCommand() const override;

  YAML::Node staticParams() const override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  bool isApplicable() override;
  bool isValid() override;

private:
  const std::map<QString, tobas::RcCommand> command_map_{
    { kRateThrottleLabel, tobas::RcCommand::kRateThrottle },
    { kAngleThrottleLabel, tobas::RcCommand::kAngleThrottle },
    { kAccelYawLabel, tobas::RcCommand::kAccelYaw },
    { kPosVelYawLabel, tobas::RcCommand::kPosVelYaw },
    { kAccelRateLabel, tobas::RcCommand::kAccelRate },
    { kAccelAngleLabel, tobas::RcCommand::kAccelAngle },
    { kPosVelAngleLabel, tobas::RcCommand::kPosVelAngle },
    { kSpeedRollDeltaPitchLabel, tobas::RcCommand::kSpeedRollDPitch },
  };

  ParamGetterWidget_LineEdit* package_;
  ParamGetterWidget_LineEdit* plugin_;
  ParamGetterWidget_ComboBox* acrobat_mode_;
  ParamGetterWidget_ComboBox* stabilize_mode_;
  ParamGetterWidget_ComboBox* loiter_mode_;
};
}  // namespace sa
}  // namespace gui
