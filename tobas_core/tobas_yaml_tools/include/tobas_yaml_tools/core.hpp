#pragma once

#include <iostream>
#include <filesystem>
#include <yaml-cpp/yaml.h>

namespace yaml
{
template <typename T>
bool load(const std::string& key, const YAML::Node& parent, T& value)
{
  if (!parent[key].IsDefined())
  {
    std::cerr << "Key \"" << key << "\" is not defined." << std::endl;
    return false;
  }

  try
  {
    value = parent[key].as<T>();
  }
  catch (...)
  {
    std::cerr << "Key \"" << key << "\" type mismatch." << std::endl;
    return false;
  }

  return true;
}

bool load(const std::filesystem::path& path, YAML::Node& node);
bool save(const std::filesystem::path& path, const YAML::Node& node);
}  // namespace yaml
