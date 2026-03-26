#pragma once

#include <tobas_qt_tools/widgets/combo_box.hpp>

#include "./base.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace ctrl
{
class CustomFrameWidget : public BaseControllerWidget
{
  Q_OBJECT

  static constexpr char kAcrobatLabel[] = "Acrobat Mode";
  static constexpr char kStabilizeLabel[] = "Stabilize Mode";
  static constexpr char kLoiterLabel[] = "Loiter Mode";

  static constexpr char kRateThrottleLabel[] = "Angle Rate + Throttle";
  static constexpr char kRateThrottleVectorLabel[] = "Angle Rate + Throttle + Thrust Direction";
  static constexpr char kAngleThrottleLabel[] = "Euler Angle + Throttle";
  static constexpr char kAngleThrottleVectorLabel[] = "Euler Angle + Throttle + Thrust Direction";
  static constexpr char kAccelYawLabel[] = "Accel + Yaw";
  static constexpr char kAccelPitchYawLabel[] = "Accel + Pitch + Yaw";
  static constexpr char kPosVelAccYawLabel[] = "Position + Velocity + Yaw";
  static constexpr char kPosVelAccPitchYawLabel[] = "Position + Velocity + Pitch + Yaw";
  static constexpr char kAccelRateLabel[] = "Accel + Angle Rate";
  static constexpr char kAccelAngleLabel[] = "Accel + Euler Angle";
  static constexpr char kPosVelAccAngleLabel[] = "Position + Velocity + Angle";
  static constexpr char kSpeedRollDeltaPitchLabel[] = "Speed + Roll + Pitch";

public:
  explicit CustomFrameWidget();

  FrameType frameType() const override;
  QString controllerPackage() const override;
  QString pluginName() const override;

  tobas::RcCommand acrobatModeCommand() const override;
  tobas::RcCommand stabilizeModeCommand() const override;
  tobas::RcCommand loiterModeCommand() const override;

  YAML::Node staticParams() const override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  bool isValid() override;

private:
  const std::map<QString, tobas::RcCommand> command_map_{
    { kRateThrottleLabel, tobas::RcCommand::kRateThrottle },
    { kRateThrottleVectorLabel, tobas::RcCommand::kRateThrottleVector },
    { kAngleThrottleLabel, tobas::RcCommand::kAngleThrottle },
    { kAngleThrottleVectorLabel, tobas::RcCommand::kAngleThrottleVector },
    { kAccelYawLabel, tobas::RcCommand::kAccelYaw },
    { kAccelPitchYawLabel, tobas::RcCommand::kAccelPitchYaw },
    { kPosVelAccYawLabel, tobas::RcCommand::kPosVelAccYaw },
    { kPosVelAccPitchYawLabel, tobas::RcCommand::kPosVelAccPitchYaw },
    { kAccelRateLabel, tobas::RcCommand::kAccelRate },
    { kAccelAngleLabel, tobas::RcCommand::kAccelAngle },
    { kPosVelAccAngleLabel, tobas::RcCommand::kPosVelAccAngle },
    { kSpeedRollDeltaPitchLabel, tobas::RcCommand::kSpeedRollDPitch },
  };

  tobas::qt::ComboBox* acrobat_mode_;
  tobas::qt::ComboBox* stabilize_mode_;
  tobas::qt::ComboBox* loiter_mode_;
};
}  // namespace ctrl
}  // namespace sa
}  // namespace gui
}  // namespace tobas
