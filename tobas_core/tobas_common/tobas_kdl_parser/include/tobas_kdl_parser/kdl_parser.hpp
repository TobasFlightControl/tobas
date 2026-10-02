// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

#include <urdf_model/types.h>

#include <tobas_kdl/tree.hpp>
#include <tobas_urdf/parser.hpp>

namespace tobas
{
namespace kdl
{
class TreeParser
{
public:
  explicit TreeParser();

  std::expected<Tree, std::string> parseFromPath(const std::string& path);
  std::expected<Tree, std::string> parseFromText(const std::string& xml);
  std::expected<Tree, std::string> parseFromUrdf(const ::urdf::ModelInterface& model);

private:
  urdf::Parser urdf_parser_;

  /** Recursive function to walk through tree. */
  static void addChildrenToTree(const ::urdf::LinkConstSharedPtr& root, Tree& tree);
};
}  // namespace kdl
}  // namespace tobas
