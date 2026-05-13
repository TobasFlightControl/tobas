// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_mission_items/altitude_frame.hpp"

#include <iostream>

#define RELATIVE_TO_LAUNCH_TEXT "relative_to_launch"
#define MEAN_SEA_LEVEL_TEXT "mean_sea_level"

namespace YAML
{
Node convert<tobas::mission::AltitudeFrame>::encode(const tobas::mission::AltitudeFrame& rhs)
{
  switch (rhs) {
    case tobas::mission::AltitudeFrame::kRelativeToLaunch:
      return Node(RELATIVE_TO_LAUNCH_TEXT);
    case tobas::mission::AltitudeFrame::kMeanSeaLevel:
      return Node(MEAN_SEA_LEVEL_TEXT);
    default:
      throw;
  }
}

bool convert<tobas::mission::AltitudeFrame>::decode(const Node& node, tobas::mission::AltitudeFrame& rhs)
{
  if (!node.IsScalar()) {
    return false;
  }

  const auto text = node.as<std::string>();

  if (text == RELATIVE_TO_LAUNCH_TEXT) {
    rhs = tobas::mission::AltitudeFrame::kRelativeToLaunch;
    return true;
  }
  else if (text == MEAN_SEA_LEVEL_TEXT) {
    rhs = tobas::mission::AltitudeFrame::kMeanSeaLevel;
    return true;
  }
  else {
    std::cerr << "Invalid altitude frame: " << text << std::endl;
    return false;
  }
}
}  // namespace YAML
