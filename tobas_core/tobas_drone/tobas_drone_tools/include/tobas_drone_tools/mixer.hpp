#pragma once

#include <tobas_drone_core/drone.hpp>
#include <tobas_kdl/tree.hpp>

namespace tobas
{
/**
 * @brief ミキサーの基底クラス．
 */
class Mixer
{
  static constexpr double kZeroThrustThresh = 1e-2;  // [N]

public:
  explicit Mixer(const Drone& drone, const kdl::Tree& tree);

  virtual bool updateInternalDataStructures();

  bool setRotorLiveliness(const std::string& link_name, bool alive);

  inline bool isInitialized() const;

protected:
  const Drone& drone_;
  const kdl::Tree& tree_;

  std::map<std::string, bool> rotor_alive_;

  /* 閾値未満の微小推力をゼロにする． */
  inline double thrustDeadband(double thrust) const;

private:
  bool is_initialized_ = false;
};

inline bool Mixer::isInitialized() const
{
  return is_initialized_;
}

inline double Mixer::thrustDeadband(double thrust) const
{
  return thrust < kZeroThrustThresh ? 0. : thrust;
}
}  // namespace tobas
