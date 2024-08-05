#pragma once

namespace tobas
{
struct BatteryConfig
{
  double nominal_voltage;
  double max_voltage;
  double sag_voltage;
  double max_current;
};
}  // namespace tobas
