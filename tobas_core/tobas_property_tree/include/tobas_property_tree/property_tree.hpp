// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <filesystem>
#include <iostream>
#include <ranges>

#include <boost/property_tree/ptree.hpp>

namespace tobas
{
namespace ptree
{
class PropertyTree
{
public:
  explicit PropertyTree();

  bool initialize(const std::filesystem::path& file_path);
  bool save();

  template <typename T>
  bool get(const std::string& key, T& dst) const;
  template <typename T>
  void set(const std::string& key, const T& src);

  template <typename T>
  bool get(const std::string& key, std::vector<T>& dst) const;
  template <typename T>
  void set(const std::string& key, const std::vector<T>& src);

  template <typename T, size_t N>
  bool get(const std::string& key, std::array<T, N>& dst) const;
  template <typename T, size_t N>
  void set(const std::string& key, const std::array<T, N>& src);

  template <typename T>
  bool get(const std::string& section, const std::string& key, T& dst) const;
  template <typename T>
  void set(const std::string& section, const std::string& key, const T& src);

  inline const std::filesystem::path& filePath() const;

private:
  std::filesystem::path file_path_;
  std::filesystem::path parent_dir_;

  boost::property_tree::ptree root_node_;

  bool erase(boost::property_tree::ptree& node, boost::property_tree::path path);

  static std::string sectionedKey(const std::string& section, const std::string& key);
};

template <typename T>
bool PropertyTree::get(const std::string& key, T& dst) const
{
  const auto value = root_node_.get_optional<T>(key);
  if (!value) {
    return false;
  }

  dst = *value;
  return true;
}

template <typename T>
void PropertyTree::set(const std::string& key, const T& src)
{
  root_node_.put(key, src);
}

template <typename T>
bool PropertyTree::get(const std::string& key, std::vector<T>& dst) const
{
  const auto list_node = root_node_.get_child_optional(key);
  if (!list_node) {
    return false;
  }

  dst.clear();
  for (const auto& [_, elem_node] : *list_node) {
    const auto value = elem_node.get_optional<T>("");
    if (!value) {
      return false;
    }

    dst.push_back(*value);
  }

  return true;
}

template <typename T>
void PropertyTree::set(const std::string& key, const std::vector<T>& src)
{
  boost::property_tree::ptree list_node;

  for (const auto& elem : src) {
    boost::property_tree::ptree elem_node;
    elem_node.put("", elem);
    list_node.push_back(std::make_pair("", elem_node));
  }

  root_node_.put_child(key, list_node);
}

template <typename T, size_t N>
bool PropertyTree::get(const std::string& key, std::array<T, N>& dst) const
{
  const auto list_node = root_node_.get_child_optional(key);
  if (!list_node) {
    return false;
  }

  if (list_node->size() != N) {
    std::cerr << "Property tree list node size mismatch: " << list_node->size() << " != " << N << std::endl;
    return false;
  }

  for (const auto& [idx, item] : std::views::enumerate(*list_node)) {
    const auto& elem_node = item.second;
    const auto value = elem_node.get_optional<T>("");
    if (!value) {
      return false;
    }

    dst.at(idx) = *value;
  }

  return true;
}

template <typename T, size_t N>
void PropertyTree::set(const std::string& key, const std::array<T, N>& src)
{
  boost::property_tree::ptree list_node;

  for (const auto& elem : src) {
    boost::property_tree::ptree elem_node;
    elem_node.put("", elem);
    list_node.push_back(std::make_pair("", elem_node));
  }

  root_node_.put_child(key, list_node);
}

template <typename T>
bool PropertyTree::get(const std::string& section, const std::string& key, T& dst) const
{
  return get(sectionedKey(section, key), dst);
}

template <typename T>
void PropertyTree::set(const std::string& section, const std::string& key, const T& src)
{
  set(sectionedKey(section, key), src);
}

inline const std::filesystem::path& PropertyTree::filePath() const
{
  return file_path_;
}
}  // namespace ptree
}  // namespace tobas
