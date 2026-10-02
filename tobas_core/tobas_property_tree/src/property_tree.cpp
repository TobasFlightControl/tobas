// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_property_tree/property_tree.hpp"

#include <boost/property_tree/json_parser.hpp>

namespace fs = std::filesystem;

namespace tobas
{
namespace ptree
{
PropertyTree::PropertyTree()
{
}

std::expected<void, std::string> PropertyTree::initialize(const fs::path& file_path)
{
  if (file_path.empty()) {
    return std::unexpected("Invalid file path.");
  }

  // Load the file if it exists.
  if (fs::is_regular_file(file_path)) {
    try {
      boost::property_tree::json_parser::read_json(file_path, root_node_);
    }
    catch (const std::exception& e) {
      return std::unexpected("Read json failed: " + std::string(e.what()));
    }
  }

  file_path_ = file_path;
  parent_dir_ = file_path.parent_path();

  return {};
}

std::expected<void, std::string> PropertyTree::save()
{
  if (file_path_.empty()) {
    return std::unexpected("Property tree is not initialized yet.");
  }

  // Create the parent directory if it does not exist.
  if (!fs::is_directory(parent_dir_)) {
    if (!fs::create_directories(parent_dir_)) {
      return std::unexpected("Failed to create directory: " + parent_dir_.string());
    }
  }

  try {
    boost::property_tree::json_parser::write_json(file_path_, root_node_);
  }
  catch (const std::exception& e) {
    return std::unexpected("Write json failed: " + std::string(e.what()));
  }

  return {};
}

std::string PropertyTree::sectionedKey(const std::string& section, const std::string& key)
{
  if (section.empty()) {
    return key;
  }
  else {
    return section + "." + key;
  }
}
}  // namespace ptree
}  // namespace tobas
