// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_tools/mixer_i.hpp"

namespace tobas
{
MixerI::MixerI(const Drone& drone, const kdl::Tree& tree) : drone_(drone), tree_(tree)
{
}

std::expected<void, std::string> MixerI::updateInternalDataStructures()
{
  if (!drone_.isValid()) {
    return std::unexpected("Drone configuration is invalid.");
  }

  rotor_alive_.clear();
  for (const auto& [link_name, _] : drone_.prop->rotors) {
    rotor_alive_[link_name] = true;
  }

  return {};
}

std::expected<void, std::string> MixerI::setRotorLiveliness(const std::string& link_name, bool alive)
{
  if (!isInitialized()) {
    return std::unexpected("Mixer is not initialized.");
  }

  const auto it = rotor_alive_.find(link_name);
  if (it == rotor_alive_.end()) {
    return std::unexpected("Invalid rotor link name: " + link_name);
  }

  it->second = alive;
  return {};
}
}  // namespace tobas
