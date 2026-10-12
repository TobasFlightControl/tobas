// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_tools/util.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_path_tools/join.hpp>

namespace tobas
{
std::string addThrotNS(const std::string& _topic)
{
  return path::join(kThrottledNS, _topic);
}

std::string addIfaceNS(const std::string& _topic)
{
  return path::join(kRemoteIfaceNS, _topic);
}
}  // namespace tobas
