// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <filesystem>
#include <ranges>
#include <string>

#include <boost/property_tree/ptree.hpp>

namespace tobas
{
namespace ptree
{
class PropertyTree
{
public:
  explicit PropertyTree();

  std::expected<void, std::string> initialize(const std::filesystem::path& _file_path);
  std::expected<void, std::string> save();

  template <typename T>
  bool get(const std::string& _key, T& _dst) const;
  template <typename T>
  void set(const std::string& _key, const T& _src);

  template <typename T>
  bool get(const std::string& _key, std::vector<T>& _dst) const;
  template <typename T>
  void set(const std::string& _key, const std::vector<T>& _src);

  template <typename T, size_t N>
  bool get(const std::string& _key, std::array<T, N>& _dst) const;
  template <typename T, size_t N>
  void set(const std::string& _key, const std::array<T, N>& _src);

  template <typename T>
  bool get(const std::string& _section, const std::string& _key, T& _dst) const;
  template <typename T>
  void set(const std::string& _section, const std::string& _key, const T& _src);

  inline const std::filesystem::path& filePath() const;

private:
  std::filesystem::path file_path_;
  std::filesystem::path parent_dir_;

  boost::property_tree::ptree root_node_;

  static std::string sectionedKey(const std::string& _section, const std::string& _key);
};

template <typename T>
bool PropertyTree::get(const std::string& _key, T& _dst) const
{
  const auto value = root_node_.get_optional<T>(_key);
  if (!value) {
    return false;
  }

  _dst = *value;
  return true;
}

template <typename T>
void PropertyTree::set(const std::string& _key, const T& _src)
{
  root_node_.put(_key, _src);
}

template <typename T>
bool PropertyTree::get(const std::string& _key, std::vector<T>& _dst) const
{
  const auto list_node = root_node_.get_child_optional(_key);
  if (!list_node) {
    return false;
  }

  _dst.clear();
  for (const auto& [_, elem_node] : *list_node) {
    const auto value = elem_node.get_optional<T>("");
    if (!value) {
      return false;
    }

    _dst.push_back(*value);
  }

  return true;
}

template <typename T>
void PropertyTree::set(const std::string& _key, const std::vector<T>& _src)
{
  boost::property_tree::ptree list_node;

  for (const auto& elem : _src) {
    boost::property_tree::ptree elem_node;
    elem_node.put("", elem);
    list_node.push_back(std::make_pair("", elem_node));
  }

  root_node_.put_child(_key, list_node);
}

template <typename T, size_t N>
bool PropertyTree::get(const std::string& _key, std::array<T, N>& _dst) const
{
  const auto list_node = root_node_.get_child_optional(_key);
  if (!list_node) {
    return false;
  }

  if (list_node->size() != N) {
    return false;
  }

  for (const auto& [idx, item] : std::views::enumerate(*list_node)) {
    const auto& elem_node = item.second;
    const auto value = elem_node.get_optional<T>("");
    if (!value) {
      return false;
    }

    _dst.at(idx) = *value;
  }

  return true;
}

template <typename T, size_t N>
void PropertyTree::set(const std::string& _key, const std::array<T, N>& _src)
{
  boost::property_tree::ptree list_node;

  for (const auto& elem : _src) {
    boost::property_tree::ptree elem_node;
    elem_node.put("", elem);
    list_node.push_back(std::make_pair("", elem_node));
  }

  root_node_.put_child(_key, list_node);
}

template <typename T>
bool PropertyTree::get(const std::string& _section, const std::string& _key, T& _dst) const
{
  return get(sectionedKey(_section, _key), _dst);
}

template <typename T>
void PropertyTree::set(const std::string& _section, const std::string& _key, const T& _src)
{
  set(sectionedKey(_section, _key), _src);
}

inline const std::filesystem::path& PropertyTree::filePath() const
{
  return file_path_;
}
}  // namespace ptree
}  // namespace tobas
