#pragma once

#include <tobas_drone_core/drone.hpp>

#include "./solver_i.hpp"

namespace tobas
{
/**
 * @brief 特定の回転軸を持つロータを抽出する．
 */
class RotorAxisExtractor : public SolverI
{
public:
  explicit RotorAxisExtractor(const Drone& drone, const rotor_axis_t& axis);

  bool updateInternalDataStructures() override;

  inline size_t count() const;

  inline const std::string& linkName(size_t idx) const;
  inline RotorConfig::ConstSharedPtr rotor(size_t idx) const;

  double maxThrustSum() const;

private:
  const Drone& drone_;
  const rotor_axis_t axis_;

  std::vector<std::string> link_names_;

  void initialize();
};

inline size_t RotorAxisExtractor::count() const
{
  return link_names_.size();
}

inline const std::string& RotorAxisExtractor::linkName(size_t idx) const
{
  assert(idx < count());
  return link_names_[idx];
}

inline RotorConfig::ConstSharedPtr RotorAxisExtractor::rotor(size_t idx) const
{
  return drone_.prop->rotors.at(linkName(idx));
}
}  // namespace tobas
