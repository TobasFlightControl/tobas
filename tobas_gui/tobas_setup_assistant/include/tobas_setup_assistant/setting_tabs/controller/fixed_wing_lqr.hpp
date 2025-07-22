#pragma once

#include "./base.hpp"
#include "tobas_setup_assistant/robot_info.hpp"

namespace gui
{
namespace sa
{
class FixedWingLQRWidget : public BaseControllerWidget
{
  Q_OBJECT

  static constexpr int kMinNumProp = 1;
  static constexpr int kMinNumCS = 2;

public:
  explicit FixedWingLQRWidget(RobotInfo& robot);

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
  RobotInfo& robot_;
};
}  // namespace sa
}  // namespace gui
