// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_mission_items/altitude_frame.hpp"

#include <iostream>

namespace YAML
{
namespace
{
constexpr char kRelativeToLaunchText[] = "relative_to_launch";
constexpr char kMeanSeaLevelText[] = "mean_sea_level";
}  // namespace

Node convert<tobas::mission::AltitudeFrame>::encode(const tobas::mission::AltitudeFrame& rhs)
{
  switch (rhs) {
    case tobas::mission::AltitudeFrame::kRelativeToLaunch:
      return Node(kRelativeToLaunchText);
    case tobas::mission::AltitudeFrame::kMeanSeaLevel:
      return Node(kMeanSeaLevelText);
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

  if (text == kRelativeToLaunchText) {
    rhs = tobas::mission::AltitudeFrame::kRelativeToLaunch;
    return true;
  }
  else if (text == kMeanSeaLevelText) {
    rhs = tobas::mission::AltitudeFrame::kMeanSeaLevel;
    return true;
  }
  else {
    std::cerr << "Invalid altitude frame: " << text << std::endl;
    return false;
  }
}
}  // namespace YAML
