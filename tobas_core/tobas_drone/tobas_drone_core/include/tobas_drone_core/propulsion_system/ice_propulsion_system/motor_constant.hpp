#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
/* ct = c0 + c1 φ */
class VppMotorConstant
{
  static constexpr char kC0Key[] = "c0";
  static constexpr char kC1Key[] = "c1";

public:
  double c0;
  double c1;

  bool isValid() const;

  bool load(const YAML::Node& node);
  YAML::Node dump() const;

  inline double compute(double phi) const
  {
    return c0 + c1 * phi;
  }
};
}  // namespace tobas
