#pragma once

#include "./base.hpp"

namespace gui
{
namespace sa
{
class T1Widget : public BaseHardwareWidget
{
  Q_OBJECT

public:
  explicit T1Widget();

  const char* name() const override;
  const char* description() const override;
  const char* hardwarePackage() const override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  bool isValid() override;
};
}  // namespace sa
}  // namespace gui
