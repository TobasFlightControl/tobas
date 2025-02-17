#include "../include/tobas_drone_tools/mixer.hpp"

using namespace std;

namespace tobas
{
Mixer::Mixer(const Drone& drone, const kdl::Tree& tree) : drone_(drone), tree_(tree)
{
}

bool Mixer::updateInternalDataStructures()
{
  rotor_alive_.clear();
  for (const auto& [channel, _] : drone_.rotors)
    rotor_alive_[channel] = true;

  return true;
}

bool Mixer::setRotorLiveliness(size_t channel, bool alive)
{
  if (!rotor_alive_.contains(channel))
  {
    cerr << "Invalid rotor channel: " << channel << endl;
    return false;
  }

  rotor_alive_[channel] = alive;
  return true;
}
}  // namespace tobas
