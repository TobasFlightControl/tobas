// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_property_tree/property_tree.hpp"

#include <boost/property_tree/json_parser.hpp>

#include <tobas_path_tools/core.hpp>

using namespace std;
namespace fs = std::filesystem;

namespace tobas
{
namespace ptree
{
PropertyTree::PropertyTree()
{
}

bool PropertyTree::initialize(const fs::path& file_path)
{
  if (fs::is_regular_file(file_path)) {
    // ファイルが存在する場合は読み込む
    try {
      boost::property_tree::json_parser::read_json(file_path, root_node_);
      cout << file_path << " is loaded successfully." << endl;
    }
    catch (const exception& e) {
      // 読み込みに失敗したら元のファイルを削除
      cerr << "Failed to load " << file_path << ": " << e.what() << endl;
      cerr << "Removing " << file_path << "." << endl;
      if (!fs::remove(file_path)) {
        cerr << "Failed to remove " << file_path << "." << endl;
        return false;
      }
    }
  }
  else {
    cout << file_path << " does not exist." << endl;
  }

  file_path_ = file_path;
  parent_dir_ = file_path.parent_path();

  return true;
}

bool PropertyTree::save()
{
  // 親ディレクトリが存在しない場合は作成する
  if (!fs::is_directory(parent_dir_)) {
    if (!fs::create_directories(parent_dir_)) {
      cerr << "Failed to create " << parent_dir_ << "." << endl;
      return false;
    }
  }

  try {
    boost::property_tree::json_parser::write_json(file_path_, root_node_);
  }
  catch (const exception& e) {
    cerr << "Failed to save " << file_path_ << ": " << e.what() << endl;
    return false;
  }

  return true;
}

bool PropertyTree::erase(boost::property_tree::ptree& node, boost::property_tree::path path)
{
  if (path.empty()) {
    cerr << "Path is empty." << endl;
    return false;
  }

  // パスの先頭要素を取得
  const auto child_name = path.reduce();

  // 名前空間がなければ要素を消して終了
  if (path.empty()) {
    if (node.erase(child_name) == 0) {
      cerr << "Failed to erase key \"" << child_name << "\"." << endl;
      return false;
    }
    return true;
  }

  // 入れ子になってたら再帰的に呼び出す
  auto child_node_opt = node.get_child_optional(child_name);
  if (!child_node_opt) {
    cerr << "Failed to get child node \"" << child_name << "\"." << endl;
    return false;
  }

  auto& child_node = child_node_opt.get();
  return erase(child_node, path);
}

string PropertyTree::sectionedKey(const string& section, const string& key)
{
  if (section.empty()) {
    return key;
  }
  else {
    return section + "." + key;
  }
}
}  // namespace ptree
}  // namespace tobas
