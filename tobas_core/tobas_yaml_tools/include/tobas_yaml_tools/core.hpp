#pragma once

#include <iostream>
#include <filesystem>
#include <yaml-cpp/yaml.h>

namespace yaml
{
template <typename T>
bool load(const std::string& key, const YAML::Node& parent, T& value)
{
  if (!parent.IsMap()) {
    std::cerr << "The type of the parent node of key \"" << key << "\" is not map." << std::endl;
    return false;
  }

  try {
    value = parent[key].as<T>();
  }
  catch (...) {
    std::cerr << "Key \"" << key << "\" type mismatch." << std::endl;
    return false;
  }

  return true;
}

/* YAML::Nodeをテキストに変換する． */
std::string dump(const YAML::Node& node);

bool load(const std::filesystem::path& path, YAML::Node& node);
bool save(const std::filesystem::path& path, const YAML::Node& node);
}  // namespace yaml
