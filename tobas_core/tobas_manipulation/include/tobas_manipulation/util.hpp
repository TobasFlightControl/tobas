// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_msgs_adapter/link_state_array.hpp>

namespace tobas
{
namespace manipulation
{
std::vector<std::string> linkNames(const tobas_msgs::LinkStateArray& msg);
}  // namespace manipulation
}  // namespace tobas
