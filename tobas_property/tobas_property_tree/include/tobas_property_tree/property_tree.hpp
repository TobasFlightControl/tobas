#pragma once

#include <filesystem>
#include <iostream>
#include <ranges>

#include <boost/property_tree/ptree.hpp>

namespace ptree
{
class PropertyTree
{
public:
  explicit PropertyTree();

  bool initialize(const std::filesystem::path& file_path);
  bool save();

  template <typename T>
  bool get(const std::string& key, T& value) const;
  template <typename T>
  void set(const std::string& key, const T& value);

  template <typename T>
  bool get(const std::string& key, std::vector<T>& vec) const;
  template <typename T>
  void set(const std::string& key, const std::vector<T>& vec);

  template <typename T, size_t N>
  bool get(const std::string& key, std::array<T, N>& arr) const;
  template <typename T, size_t N>
  void set(const std::string& key, const std::array<T, N>& arr);

  inline const std::filesystem::path& filePath() const;

private:
  std::filesystem::path file_path_;
  std::filesystem::path parent_dir_;

  boost::property_tree::ptree root_node_;
};

template <typename T>
bool PropertyTree::get(const std::string& key, T& value) const
{
  const auto opt = root_node_.get_optional<T>(key);
  if (!opt) {
    return false;
  }

  value = opt.get();
  return true;
}

template <typename T>
void PropertyTree::set(const std::string& key, const T& value)
{
  root_node_.put(key, value);
}

template <typename T>
bool PropertyTree::get(const std::string& key, std::vector<T>& vec) const
{
  const auto list_node_opt = root_node_.get_child_optional(key);
  if (!list_node_opt) {
    return false;
  }

  vec.clear();
  for (const auto& [_, elem_node] : list_node_opt.get()) {
    const auto value_opt = elem_node.get_optional<T>("");
    if (!value_opt) {
      return false;
    }

    vec.push_back(value_opt.get());
  }

  return true;
}

template <typename T>
void PropertyTree::set(const std::string& key, const std::vector<T>& vec)
{
  boost::property_tree::ptree list_node;

  for (const auto& value : vec) {
    boost::property_tree::ptree elem_node;
    elem_node.put("", value);
    list_node.push_back(std::make_pair("", elem_node));
  }

  root_node_.erase(key);
  root_node_.add_child(key, list_node);
}

template <typename T, size_t N>
bool PropertyTree::get(const std::string& key, std::array<T, N>& arr) const
{
  const auto list_node_opt = root_node_.get_child_optional(key);
  if (!list_node_opt) {
    return false;
  }

  const auto list_node = list_node_opt.get();
  if (list_node.size() != N) {
    std::cerr << "Property tree list node size mismatch: " << list_node.size() << " != " << N << std::endl;
    return false;
  }

  for (const auto& [idx, item] : std::views::enumerate(list_node)) {
    const auto& elem_node = item.second;
    const auto value_opt = elem_node.get_optional<T>("");
    if (!value_opt) {
      return false;
    }

    arr.at(idx) = value_opt.get();
  }

  return true;
}

template <typename T, size_t N>
void PropertyTree::set(const std::string& key, const std::array<T, N>& arr)
{
  boost::property_tree::ptree list_node;

  for (const auto& value : arr) {
    boost::property_tree::ptree elem_node;
    elem_node.put("", value);
    list_node.push_back(std::make_pair("", elem_node));
  }

  root_node_.erase(key);
  root_node_.add_child(key, list_node);
}

inline const std::filesystem::path& PropertyTree::filePath() const
{
  return file_path_;
}
}  // namespace ptree
