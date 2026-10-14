// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_mission_items/altitude_frame.hpp"

namespace YAML
{
namespace
{
constexpr char kRelativeToLaunchText[] = "relative_to_launch";
constexpr char kMeanSeaLevelText[] = "mean_sea_level";
}  // namespace

Node convert<tobas::mission::AltitudeFrame>::encode(tobas::mission::AltitudeFrame _rhs)
{
  switch (_rhs) {
    case tobas::mission::AltitudeFrame::kRelativeToLaunch:
      return Node(kRelativeToLaunchText);
    case tobas::mission::AltitudeFrame::kMeanSeaLevel:
      return Node(kMeanSeaLevelText);
    default:
      throw;
  }
}

bool convert<tobas::mission::AltitudeFrame>::decode(const Node& _node, tobas::mission::AltitudeFrame& _rhs)
{
  if (!_node.IsScalar()) {
    return false;
  }

  const auto text = _node.as<std::string>();

  if (text == kRelativeToLaunchText) {
    _rhs = tobas::mission::AltitudeFrame::kRelativeToLaunch;
    return true;
  }
  else if (text == kMeanSeaLevelText) {
    _rhs = tobas::mission::AltitudeFrame::kMeanSeaLevel;
    return true;
  }
  else {
    return false;
  }
}
}  // namespace YAML
