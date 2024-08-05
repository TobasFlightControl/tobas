#pragma once

namespace tobas
{
struct BatteryConfig
{
  double nominal_voltage;  // [V]
  double max_voltage;      // [V]
  double sag_voltage;      // [V]
  double max_current;      // [A]
};
}  // namespace tobas
