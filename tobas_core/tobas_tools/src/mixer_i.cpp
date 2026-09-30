// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_tools/mixer_i.hpp"

#include <cassert>

namespace tobas
{
MixerI::MixerI(const Drone& drone, const kdl::Tree& tree) : drone_(drone), tree_(tree)
{
}

void MixerI::updateInternalDataStructures()
{
  assert(drone_.validate());

  rotor_alive_.clear();
  for (const auto& [link_name, _] : drone_.prop->rotors) {
    rotor_alive_[link_name] = true;
  }
}

std::expected<void, std::string> MixerI::setRotorLiveliness(const std::string& link_name, bool alive)
{
  if (!isInitialized()) {
    return std::unexpected("Mixer is not initialized.");
  }

  const auto it = rotor_alive_.find(link_name);
  if (it == rotor_alive_.end()) {
    return std::unexpected("Rotor link name is unknown.");
  }

  it->second = alive;
  return {};
}
}  // namespace tobas
