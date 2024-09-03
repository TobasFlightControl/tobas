#pragma once

#include "./base.hpp"

namespace gui
{
namespace setup_assistant
{
class AsoWidget : public BaseHardwareWidget
{
  Q_OBJECT

public:
  const char* name() const override;
  const char* description() const override;
  const char* hardwarePackage() const override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  bool isValid() override;
};
}  // namespace setup_assistant
}  // namespace gui
