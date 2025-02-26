#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
class BatteryConfig
{
  static constexpr char kNominalVoltageKey[] = "nominal_voltage";
  static constexpr char kMaxVoltageKey[] = "max_voltage";
  static constexpr char kSagVoltageKey[] = "sag_voltage";
  static constexpr char kMaxCurrentKey[] = "max_current";

public:
  double nominal_voltage = 0.;  // [V]
  double max_voltage = 0.;      // [V]
  double sag_voltage = 0.;      // [V]
  double max_current = 0.;      // [A]

  bool isValid() const;

  bool load(const YAML::Node& node);
  YAML::Node dump() const;
};
}  // namespace tobas
