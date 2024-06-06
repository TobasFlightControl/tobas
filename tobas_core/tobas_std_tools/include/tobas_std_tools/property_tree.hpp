#pragma once

#include <boost/property_tree/ini_parser.hpp>

#include "./console.hpp"

namespace tobas_std
{
class PropertyTree
{
public:
  explicit PropertyTree(const std::string& ini_path, const std::string& section = "DEFAULT");

  void load();
  void save();

  const std::string& configPath();

  template <typename T>
  T get(const std::string& key) const;

  template <typename T>
  T get(const std::string& key, const T& _default) const;

  template <typename T>
  bool get(const std::string& key, T& value, const T& _default) const;

  template <typename T>
  void put(const std::string& key, const T& value);

private:
  boost::property_tree::ptree pt_;
  const std::string ini_path_;
  const std::string section_;

  std::string keyWithSection(const std::string& key) const;
};

template <typename T>
T PropertyTree::get(const std::string& key) const
{
  const auto optional = pt_.get_optional<T>(keyWithSection(key));
  if (!optional)
    throw std::runtime_error("Failed to get property: " + key);
  return optional.get();
}

template <typename T>
T PropertyTree::get(const std::string& key, const T& _default) const
{
  const auto optional = pt_.get_optional<T>(keyWithSection(key));
  return optional ? optional.get() : _default;
}

template <typename T>
bool PropertyTree::get(const std::string& key, T& value, const T& _default) const
{
  const auto optional = pt_.get_optional<T>(keyWithSection(key));
  if (optional)
  {
    value = optional.get();
    return true;
  }
  else
  {
    value = _default;
    return false;
  }
}

template <typename T>
void PropertyTree::put(const std::string& key, const T& value)
{
  pt_.put(keyWithSection(key), value);
}
}  // namespace tobas_std
