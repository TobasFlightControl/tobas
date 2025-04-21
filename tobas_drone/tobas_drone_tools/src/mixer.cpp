#include "../include/tobas_drone_tools/mixer.hpp"

using namespace std;

namespace tobas
{
Mixer::Mixer(const Drone& drone, const kdl::Tree& tree) : drone_(drone), tree_(tree)
{
}

bool Mixer::updateInternalDataStructures()
{
  if (!drone_.isValid())
    return false;

  rotor_alive_.clear();
  for (const auto& [link_name, _] : drone_.prop->rotors)
    rotor_alive_[link_name] = true;

  is_initialized_ = true;
  return true;
}

bool Mixer::setRotorLiveliness(const string& link_name, bool alive)
{
  if (!rotor_alive_.contains(link_name))
  {
    cerr << "Invalid rotor link name: " << link_name << endl;
    return false;
  }

  rotor_alive_.at(link_name) = alive;
  return true;
}
}  // namespace tobas
