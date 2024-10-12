#pragma once

#include "./base.hpp"

namespace gui
{
namespace setup_assistant
{
class CustomControllerWidget : public BaseControllerWidget
{
  Q_OBJECT

public:
  explicit CustomControllerWidget();

  const char* name() const override;
  const char* description() const override;
  const char* controllerPackage() const override;
  const char* actionsPackage() const override;

  tobas::rc_command_t stabilizeModeCommand() const override;
  tobas::rc_command_t acrobatModeCommand() const override;

  YAML::Node staticParams() const override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  bool isApplicable() override;
  bool isValid() override;
};
}  // namespace setup_assistant
}  // namespace gui
