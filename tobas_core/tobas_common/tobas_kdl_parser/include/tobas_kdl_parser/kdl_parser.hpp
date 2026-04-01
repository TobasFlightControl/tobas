// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

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

  bool parseFromPath(const std::string& path, Tree& tree);
  bool parseFromText(const std::string& xml, Tree& tree);
  bool parseFromUrdf(const ::urdf::ModelInterface& model, Tree& tree);

  const std::string& errorMessage() const;

private:
  std::string error_msg_;

  urdf::Parser urdf_parser_;

  /* Recursive function to walk through tree. */
  static void addChildrenToTree(const ::urdf::LinkConstSharedPtr& root, Tree& tree);
};
}  // namespace kdl
}  // namespace tobas
