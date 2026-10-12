// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>
#include <vector>

#include <tobas_kdl/tree.hpp>
#include <tobas_msgs_adapter/link_state_array.hpp>

namespace tobas
{
namespace manipulation
{
std::vector<std::string> linkNames(const tobas_msgs::LinkStateArray& msg);

/**
 * Return unique movable joint names along the paths from endpoints to the root.
 * Return an error if an endpoint does not exist in the tree.
 */
std::expected<std::vector<std::string>, std::string>
findActiveJointNames(const kdl::Tree& tree, const std::vector<std::string>& endpoints);
}  // namespace manipulation
}  // namespace tobas
