#pragma once

#include <filesystem>
#include <boost/property_tree/ini_parser.hpp>

namespace ptree
{
class PropertyTree
{
  static constexpr char kDefaultSection[] = "DEFAULT";

public:
  explicit PropertyTree();

  bool initialize(const std::filesystem::path& file_path);
  bool save();

  template <typename T>
  bool get(const std::string& section, const std::string& key, T& value) const;
  template <typename T>
  void set(const std::string& section, const std::string& key, const T& value);

  template <typename T>
  bool get(const std::string& key, T& value) const;
  template <typename T>
  void set(const std::string& key, const T& value);

  inline const std::filesystem::path& filePath() const;

private:
  std::filesystem::path file_path_;
  boost::property_tree::ptree pt_;

  static inline std::string keyWithSection(const std::string& section, const std::string& key);
};

template <typename T>
bool PropertyTree::get(const std::string& section, const std::string& key, T& value) const
{
  const auto optional = pt_.get_optional<T>(keyWithSection(section, key));
  if (!optional)
    return false;

  value = optional.get();
  return true;
}

template <typename T>
void PropertyTree::set(const std::string& section, const std::string& key, const T& value)
{
  pt_.put(keyWithSection(section, key), value);
}

template <typename T>
bool PropertyTree::get(const std::string& key, T& value) const
{
  return get(kDefaultSection, key, value);
}

template <typename T>
void PropertyTree::set(const std::string& key, const T& value)
{
  set(kDefaultSection, key, value);
}

inline const std::filesystem::path& PropertyTree::filePath() const
{
  return file_path_;
}

inline std::string PropertyTree::keyWithSection(const std::string& section, const std::string& key)
{
  return section + "." + key;
}
}  // namespace ptree
