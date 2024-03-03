#pragma once

#include <boost/property_tree/ini_parser.hpp>

#include "./console.hpp"

namespace tobas_std
{
class PropertyTree
{
public:
  explicit PropertyTree(const std::string& ini_path);

  void save();

  template <typename T>
  void get(const std::string& key, T& value) const;

  template <typename T>
  bool get(const std::string& key, T& value, const T& _default) const;

  template <typename T>
  void put(const std::string& key, const T& value);

private:
  boost::property_tree::ptree pt_;
  const std::string ini_path_;
};

template <typename T>
void PropertyTree::get(const std::string& key, T& value) const
{
  const auto optional = pt_.get_optional<T>(key);
  if (!optional)
    throw std::runtime_error("Failed to get property: " + key);

  value = optional.get();
}

template <typename T>
bool PropertyTree::get(const std::string& key, T& value, const T& _default) const
{
  const auto optional = pt_.get_optional<T>(key);
  if (!optional)
  {
    value = _default;
    TOBAS_WARN("Failed to get '" << key << "'. The default '" << _default << "' is used.");
    return false;
  }

  value = optional.get();
  return true;
}

template <typename T>
void PropertyTree::put(const std::string& key, const T& value)
{
  pt_.put(key, value);
}
}  // namespace tobas_std
