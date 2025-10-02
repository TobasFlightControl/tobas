#pragma once

#include <cmath>

#include <yaml-cpp/yaml.h>

namespace tobas
{
/* cm = a (φ-φ0) + b + c / (φ-φ0) */
class VppMomentConstant
{
  static constexpr char kAKey[] = "a";
  static constexpr char kBKey[] = "b";
  static constexpr char kCKey[] = "c";
  static constexpr char kPhi0Key[] = "phi0";

public:
  double a;
  double b;
  double c;
  double phi0;  // 負の失速角 (= 推力がゼロになるピッチ角) [rad]

  bool isValid() const;

  bool load(const YAML::Node& node);
  YAML::Node dump() const;

  double compute(double phi) const;

  /* 最も効率の良いピッチ角 [rad] */
  inline double optimalPitch() const
  {
    return phi0 + sqrt(c / a);
  }
};
}  // namespace tobas
