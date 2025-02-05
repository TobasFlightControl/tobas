#pragma once

#include <tobas_kdl/tree.hpp>
#include <tobas_drone_core/drone.hpp>

namespace tobas
{
/**
 * @brief ミキサーの基底クラス．
 */
class Mixer
{
public:
  explicit Mixer(const Drone& drone, const kdl::Tree& tree);

  virtual bool updateInternalDataStructures();

  bool setRotorLiveliness(size_t channel, bool alive);

protected:
  const Drone& drone_;
  const kdl::Tree& tree_;

  std::map<size_t, bool> rotor_alive_;
};
}  // namespace tobas
