// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_tools/util.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_path_tools/join.hpp>

namespace tobas
{
std::string addThrotNS(const std::string& topic)
{
  return path::join(kThrottledNS, topic);
}

std::string addIfaceNS(const std::string& topic)
{
  return path::join(kRemoteIfaceNS, topic);
}
}  // namespace tobas
