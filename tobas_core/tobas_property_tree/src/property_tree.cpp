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

bool PropertyTree::initialize(const fs::path& file_path)
{
  if (fs::is_regular_file(file_path)) {
    // Load the file if it exists.
    try {
      boost::property_tree::json_parser::read_json(file_path, root_node_);
      std::cout << file_path << " has been loaded successfully." << std::endl;
    }
    catch (const std::exception& e) {
      // Remove the original file if loading fails.
      std::cerr << "Failed to load " << file_path << ": " << e.what() << std::endl;
      std::cerr << "Removing " << file_path << "." << std::endl;
      if (!fs::remove(file_path)) {
        std::cerr << "Failed to remove " << file_path << "." << std::endl;
        return false;
      }
    }
  }
  else {
    std::cout << file_path << " does not exist." << std::endl;
  }

  file_path_ = file_path;
  parent_dir_ = file_path.parent_path();

  return true;
}

bool PropertyTree::save()
{
  // Create the parent directory if it does not exist.
  if (!fs::is_directory(parent_dir_)) {
    if (!fs::create_directories(parent_dir_)) {
      std::cerr << "Failed to create " << parent_dir_ << "." << std::endl;
      return false;
    }
  }

  try {
    boost::property_tree::json_parser::write_json(file_path_, root_node_);
  }
  catch (const std::exception& e) {
    std::cerr << "Failed to save " << file_path_ << ": " << e.what() << std::endl;
    return false;
  }

  return true;
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
