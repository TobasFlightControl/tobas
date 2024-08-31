#pragma once

#include "./base.hpp"

namespace gui
{
namespace setup_assistant
{
class ElectrodynamicsWidget_NoSelect : public ElectrodynamicsWidget_Base
{
  Q_OBJECT

public:
  const char* name() const override;
  const char* description() const override;

  void onInit() override;

  bool isValid() override;
  void copyFrom(const ElectrodynamicsWidget_Base* src) override;

  YAML::Node dump() override;
  void load(const YAML::Node& node) override;

  std::pair<double, double> rotSpeedCoefs() const override;
};
}  // namespace setup_assistant
}  // namespace gui
