#pragma once

#include <filesystem>
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
  bool get(const std::string& key, std::vector<T>& list) const;
  template <typename T>
  void set(const std::string& key, const std::vector<T>& list);

  inline const std::filesystem::path& filePath() const;

private:
  std::filesystem::path file_path_;
  boost::property_tree::ptree root_node_;
};

template <typename T>
bool PropertyTree::get(const std::string& key, T& value) const
{
  const auto opt = root_node_.get_optional<T>(key);
  if (!opt)
    return false;

  value = opt.get();
  return true;
}

template <typename T>
void PropertyTree::set(const std::string& key, const T& value)
{
  root_node_.put(key, value);
}

template <typename T>
bool PropertyTree::get(const std::string& key, std::vector<T>& list) const
{
  const auto list_node_opt = root_node_.get_child_optional(key);
  if (!list_node_opt)
    return false;

  list.clear();

  for (const auto& [_, elem_node] : list_node_opt.get())
  {
    const auto value_opt = elem_node.get_optional<T>("");
    if (!value_opt)
      return false;

    list.push_back(value_opt.get());
  }

  return true;
}

template <typename T>
void PropertyTree::set(const std::string& key, const std::vector<T>& list)
{
  boost::property_tree::ptree list_node;

  for (const auto& value : list)
  {
    boost::property_tree::ptree elem_node;
    elem_node.put("", value);
    list_node.push_back(std::make_pair("", elem_node));
  }

  root_node_.add_child(key, list_node);
}

inline const std::filesystem::path& PropertyTree::filePath() const
{
  return file_path_;
}

}  // namespace ptree
