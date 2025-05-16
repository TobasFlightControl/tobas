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
public:
  explicit Mixer(const Drone& drone, const kdl::Tree& tree);

  virtual bool updateInternalDataStructures();

  bool setRotorLiveliness(const std::string& link_name, bool alive);

  inline bool isInitialized() const;

protected:
  const Drone& drone_;
  const kdl::Tree& tree_;

  std::map<std::string, bool> rotor_alive_;

private:
  bool is_initialized_ = false;
};

inline bool Mixer::isInitialized() const
{
  return is_initialized_;
}
}  // namespace tobas
