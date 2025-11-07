#pragma once

#include <expected>
#include <filesystem>
#include <iostream>

#include <yaml-cpp/yaml.h>

namespace yaml
{
static constexpr size_t kDefaultPrecision = 6;

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
std::expected<std::string, std::string> dump(const YAML::Node& node, size_t precision = kDefaultPrecision);

std::expected<YAML::Node, std::string> load(const std::filesystem::path& path);
bool save(const std::filesystem::path& path, const YAML::Node& node, size_t precision = kDefaultPrecision);
}  // namespace yaml
