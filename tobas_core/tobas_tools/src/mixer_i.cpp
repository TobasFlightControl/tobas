// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_tools/mixer_i.hpp"

using namespace std;

namespace tobas
{
MixerI::MixerI(const Drone& drone, const kdl::Tree& tree) : drone_(drone), tree_(tree)
{
}

bool MixerI::updateInternalDataStructures()
{
  if (!drone_.isValid()) {
    return false;
  }

  rotor_alive_.clear();
  for (const auto& [link_name, _] : drone_.prop->rotors) {
    rotor_alive_[link_name] = true;
  }

  is_initialized_ = true;
  return true;
}

bool MixerI::setRotorLiveliness(const string& link_name, bool alive)
{
  if (!rotor_alive_.contains(link_name)) {
    cerr << "Invalid rotor link name: " << link_name << endl;
    return false;
  }

  rotor_alive_.at(link_name) = alive;
  return true;
}
}  // namespace tobas
