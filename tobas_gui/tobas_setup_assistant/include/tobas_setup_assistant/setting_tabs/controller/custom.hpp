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

  RcCommand acrobatModeCommand() const override;
  RcCommand stabilizeModeCommand() const override;
  RcCommand loiterModeCommand() const override;

  YAML::Node staticParams() const override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  bool isValid() override;

private:
  const std::map<QString, RcCommand> command_map_{
    { kRateThrottleLabel, RcCommand::kRateThrottle },
    { kRateThrottleVectorLabel, RcCommand::kRateThrottleVector },
    { kAngleThrottleLabel, RcCommand::kAngleThrottle },
    { kAngleThrottleVectorLabel, RcCommand::kAngleThrottleVector },
    { kAccelYawLabel, RcCommand::kAccelYaw },
    { kAccelPitchYawLabel, RcCommand::kAccelPitchYaw },
    { kPosVelAccYawLabel, RcCommand::kPosVelAccYaw },
    { kPosVelAccPitchYawLabel, RcCommand::kPosVelAccPitchYaw },
    { kAccelRateLabel, RcCommand::kAccelRate },
    { kAccelAngleLabel, RcCommand::kAccelAngle },
    { kPosVelAccAngleLabel, RcCommand::kPosVelAccAngle },
    { kSpeedRollDeltaPitchLabel, RcCommand::kSpeedRollDPitch },
  };

  qt::ComboBox* acrobat_mode_;
  qt::ComboBox* stabilize_mode_;
  qt::ComboBox* loiter_mode_;
};
}  // namespace ctrl
}  // namespace sa
}  // namespace gui
}  // namespace tobas
